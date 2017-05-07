/* Pipeline Cache Simulator  */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

/* compatibility macro */
#define bzero(p,len) (memset((p), '\0', (len)), (void *) 0)

/* constants that affect cache size,
 * how "long" cache miss delay is,
 * and how many stages are in our pipeline.
 */
enum {
	MAX_CACHE_SIZE = 10240,
	CACHE_MISS_DELAY = 10, // 10 cycle cache miss penalty
	MAX_STAGES = 5
};

typedef unsigned int uint;
typedef unsigned char byte;

/* init the simulator */
void iplc_sim_init(int index, int blocksize, int assoc);

/* Cache simulator functions */
void iplc_sim_LRU_replace_on_miss(int index, int tag, int assoc_entry);
void iplc_sim_LRU_update_on_hit(int index, int assoc_entry);
int iplc_sim_trap_address(uint address);

/* Pipeline functions */
uint iplc_sim_parse_reg(byte *reg_str);
void iplc_sim_parse_instruction(byte *buffer);
void iplc_sim_push_pipeline_stage();
void iplc_sim_process_pipeline_rtype(byte *instruction, int dest_reg,
									 int reg1, int reg2_or_constant);
void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, uint data_address);
void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, uint data_address);
void iplc_sim_process_pipeline_branch(int reg1, int reg2);
void iplc_sim_process_pipeline_jump();
void iplc_sim_process_pipeline_syscall();
void iplc_sim_process_pipeline_nop();

/* Outout performance results */
void iplc_sim_finalize();

typedef struct cache_line{
	int valid; /* the valid bit */
	uint tag;  /* the tag */
	/* the tree structure, which provides associativity */
	struct cache_line *lru_prev, *lru_next; 
} cache_line_t;

/* When cache set is full (all valid bits in lines are set), dump tail from cache */
typedef struct cache_set{
	cache_line_t *lines, *lru_head, *lru_tail;
} cache_set_t;

cache_set_t *cache=NULL;
int cache_index=0;
int cache_blocksize=0;
int cache_blockoffsetbits = 0;
int cache_assoc=0;
long cache_miss=0;
long cache_access=0;
long cache_hit=0;

byte instruction[16];
byte reg1[16];
byte reg2[16];
byte offsetwithreg[16];
uint data_address=0;
uint instruction_address=0;
uint pipeline_cycles=0;   /* how many cycles did you pipeline consume */
uint instruction_count=0; /* home many real instructions ran thru the pipeline */
uint branch_predict_taken=0;
uint branch_count=0;
uint correct_branch_predictions=0;

uint debug=0;
uint dump_pipeline=1;

typedef struct rtype{
	byte instruction[16];
	int reg1;
	int reg2_or_constant;
	int dest_reg;
} rtype_t;

typedef struct load_word{
	uint data_address;
	int dest_reg;
	int base_reg;
} lw_t;

typedef struct store_word{
	uint data_address;
	int src_reg;
	int base_reg;
} sw_t;

typedef struct branch{
	int reg1;
	int reg2;	
} branch_t;


typedef struct jump{
	byte instruction[16];
} jump_t;

enum instruction_type {NOP, RTYPE, LW, SW, BRANCH, JUMP, JAL, SYSCALL};

typedef struct pipeline{
	enum instruction_type itype;
	uint instruction_address;
	union{
		rtype_t   rtype;
		lw_t	  lw;
		sw_t	  sw;
		branch_t  branch;
		jump_t	jump;
	} stage;
} pipeline_t;

enum pipeline_stages {FETCH, DECODE, ALU, MEM, WRITEBACK};

pipeline_t pipeline[MAX_STAGES];

/* Cache Functions */

/*
 * Correctly configure the cache.
 */
void
iplc_sim_init(int index, int blocksize, int assoc)
{
	int i=0, j=0;
	unsigned long cache_size = 0;
	cache_index = index;
	cache_blocksize = blocksize;
	cache_assoc = assoc;
	
	/* log(x)/log(2) = log_2(x)
	 * word_count * 4 bytes/word
	 * Note: rint function rounds the result up prior to casting
	 */	
	cache_blockoffsetbits = (int) rint((log( (double) (blocksize * 4) )/ log(2)));
	
	cache_size = assoc * ( 1 << index ) * ((32 * blocksize) + 33 - index - cache_blockoffsetbits);
	
	printf("Cache Configuration \n");
	printf("   Index: %d bits or %d lines \n", cache_index, (1<<cache_index) );
	printf("   BlockSize: %d \n", cache_blocksize );
	printf("   Associativity: %d \n", cache_assoc );
	printf("   BlockOffSetBits: %d \n", cache_blockoffsetbits );
	printf("   CacheSize: %lu \n", cache_size );
	
	if(cache_size > MAX_CACHE_SIZE){
		printf("Cache too big. Great than MAX SIZE of %d .... \n", MAX_CACHE_SIZE);
		exit(-1);
	}
	
	cache = (cache_set_t*) malloc(sizeof(cache_set_t) * (1<<index));
	
	// Dynamically create our cache based on the information the user entered
	for(i = 0; i < (1<<index); ++i){
		cache[i].lines = (cache_line_t*) malloc(sizeof(cache_line_t) * assoc);
		cache[i].lru_head = cache[i].lru_tail = &cache[i].lines[0];
		for(j = 0; j < assoc; ++j){
			cache[i].lines[j].valid = 0;
			cache[i].lines[j].tag = 0;
			cache[i].lines[j].lru_prev = cache[i].lines[j].lru_next = NULL;
		}
	}
	
	// init the pipeline -- set all data to zero and instructions to NOP
	for(i = 0; i < MAX_STAGES; ++i){
		// itype is set to O which is NOP type instruction
		bzero(&(pipeline[i]), sizeof(pipeline_t));
	}
}

/*
 * iplc_sim_trap_address() determined this is not in our cache. Put it there
 * and make sure that is now our Most Recently Used (MRU) entry.
 */
void
iplc_sim_LRU_replace_on_miss(int index, int assoc_entry, int tag)
{
	/*
	 * assoc != -1 means filling an unused slot
	 * assoc == -1 means replacing old data
	 */
	cache_line_t* line=NULL;
	if(assoc_entry == -1){
		/* No more unused space. Replace the oldest entry */
		line = cache[index].lru_tail;
		if(line->lru_next){
			// Keep tail valid if >1-way associative
			cache[index].lru_tail = line->lru_next;
			cache[index].lru_tail->lru_prev = NULL;
		}
	}else{
		/* Unused space at assoc_entry (determined by trap_address) */
		line = &cache[index].lines[assoc_entry];
	}

	line->valid = 1;
	line->tag = tag;

	if(line != cache[index].lru_head){
		cache[index].lru_head->lru_next = line;
		line->lru_prev = cache[index].lru_head;
		line->lru_next = NULL;
		cache[index].lru_head = line;
	}
}

/* iplc_sim_trap_address() determines the entry is in our cache.  
 * Update its information in the cache.
 */
void
iplc_sim_LRU_update_on_hit(int index, int assoc_entry)
{
	cache_line_t* line = &cache[index].lines[assoc_entry];
	
	/* Not the head, and cache_assoc > 1 */
	if(line->lru_next) {
		line->lru_next->lru_prev = line->lru_prev;

		if (line->lru_prev){
			line->lru_prev->lru_next = line->lru_next;
		}else{
			/* It's the tail, and not the head;
			 * we should set tail to the next entry before we leave.
			 */
			cache[index].lru_tail = line->lru_next;
			cache[index].lru_tail->lru_prev = NULL;
		}
		/* Make this the head */
		cache[index].lru_head->lru_next = line;
		line->lru_prev = cache[index].lru_head;
		line->lru_next = NULL;
		cache[index].lru_head = line;
	}
	/* Nothing to be done if this entry is already the head */
}

/* Check if the address is in our cache.  Update our counter statistics 
 * for cache_access, cache_hit, etc.  If our configuration supports
 * associativity we may need to check through multiple entries for our
 * desired index.  In that case we will also need to call the LRU functions.
 */
int
iplc_sim_trap_address(uint address)
{
	int i=0, index=0;
	int tag=0;
	
	uint mask = ((1 << cache_index) - 1) << cache_blockoffsetbits; // Mask to get the index bits from the address
	uint non_tag_bits = cache_blockoffsetbits + cache_index;

	index = (address & mask) >> cache_blockoffsetbits;
	tag = address >> non_tag_bits; // Extract the most significant bits

	printf("Address %x: Tag= %x, Index= %d \n", address, tag, index);

	++cache_access;
	for (; i < cache_assoc; ++i){
		if (cache[index].lines[i].valid){
			if (cache[index].lines[i].tag == tag){
				// HIT!
				++cache_hit;
				iplc_sim_LRU_update_on_hit(index, i);
				return 1;
			}
		}else{
			// Stop searching; it's not here
			++cache_miss;
			iplc_sim_LRU_replace_on_miss(index, i, tag);
			return 0;
		}
	}

	/* Out of space! Replace the oldest */
	++cache_miss;
	iplc_sim_LRU_replace_on_miss(index, -1, tag);
	return 0;
}

/* iplc_sim_finalize
 * Output the summary statistics of the simulation.
 */
void
iplc_sim_finalize()
{
	/* Finish processing all instructions in the Pipeline  */
	while (pipeline[FETCH].itype != NOP  ||
		   pipeline[DECODE].itype != NOP ||
		   pipeline[ALU].itype != NOP	||
		   pipeline[MEM].itype != NOP	||
		   pipeline[WRITEBACK].itype != NOP){
		iplc_sim_push_pipeline_stage();
	}
	
	printf(" Cache Performance \n");
	printf("\t Number of Cache Accesses is %ld \n", cache_access);
	printf("\t Number of Cache Misses is %ld \n", cache_miss);
	printf("\t Number of Cache Hits is %ld \n", cache_hit);
	printf("\t Cache Miss Rate is %f \n\n", (double)cache_miss / (double)cache_access);
	printf("Pipeline Performance \n");
	printf("\t Total Cycles is %u \n", pipeline_cycles);
	printf("\t Total Instructions is %u \n", instruction_count);
	printf("\t Total Branch Instructions is %u \n", branch_count);
	printf("\t Total Correct Branch Predictions is %u \n", correct_branch_predictions);
	printf("\t CPI is %f \n\n", (double)pipeline_cycles / (double)instruction_count);
}

/* Pipeline Functions  */
/*
 * Dump the current contents of our pipeline.
 */
void
iplc_sim_dump_pipeline()
{
	int i;
	
	for(i = 0; i < MAX_STAGES; i++){
		uint iaddr = pipeline[i].instruction_address;
		switch(i){
		case FETCH:
			printf("(cyc: %u) FETCH:\t %d: 0x%x \t", pipeline_cycles, pipeline[i].itype, iaddr);
			break;
		case DECODE:
			printf("DECODE:\t %d: 0x%x \t", pipeline[i].itype, iaddr);
			break;
		case ALU:
			printf("ALU:\t %d: 0x%x \t", pipeline[i].itype, iaddr);
			break;
		case MEM:
			printf("MEM:\t %d: 0x%x \t", pipeline[i].itype, iaddr);
			break;
		case WRITEBACK:
			printf("WB:\t %d: 0x%x \n", pipeline[i].itype, iaddr);
			break;
		default:
			printf("DUMP: Bad stage!\n" );
			exit(-1);
		}
	}
}

/* test the given instruction to see if it's immeadiate */
int
immeadiate_instruction_p(const char *instr)
{
	return strncmp(instr, "addi", 4) != 0
		&& strncmp(instr, "ori", 3) != 0
		&& strncmp(instr, "sll", 3) != 0;
}

/*
 * Check if various stages of our pipeline require stalls, forwarding, etc.
 * Then push the contents of our various pipeline stages through the pipeline.
 * record cycle count, correct branch predictions, and other data in execution.
 */
void
iplc_sim_push_pipeline_stage()
{
	int i;
	int data_hit=1;
	int cycle_count=1;
	
	/* 1. Count WRITEBACK stage is "retired" -- This I'm giving you */
	if(pipeline[WRITEBACK].instruction_address){
		instruction_count++;
		if(debug)
			printf("DEBUG: Retired Instruction at 0x%x, Type %d, at Time %u \n",
				   pipeline[WRITEBACK].instruction_address, pipeline[WRITEBACK].itype, pipeline_cycles);
	}
	
	/* 2. Check for BRANCH and correct/incorrect Branch Prediction */
	if(pipeline[DECODE].itype == BRANCH){
		int branch_taken =
			(pipeline[FETCH].instruction_address != pipeline[DECODE].instruction_address + 4) && (pipeline[FETCH].itype != NOP);
		if(branch_taken == 1){
			printf("DEBUG: Branch Taken: FETCH addr = 0x%x, DECODE instr addr = 0x%x \n",
					pipeline[FETCH].instruction_address, pipeline[DECODE].instruction_address);
		}
		if (branch_taken == branch_predict_taken){
			correct_branch_predictions++;
			// if (debug)
				
		}else{
			cycle_count = 2;
		}
		
	}
	
	switch(pipeline[MEM].itype){
	/* 3. Check for LW delays due to use in ALU stage and if data hit/miss
	 *	add delay cycles if needed.
	 */
	case LW:
		if(!iplc_sim_trap_address(pipeline[MEM].stage.lw.data_address)){
			cycle_count = CACHE_MISS_DELAY;
			printf("DATA MISS:\t Address 0x%x\n", pipeline[MEM].stage.lw.data_address);
		}else{
			printf("DATA HIT:\t Address 0x%x\n", pipeline[MEM].stage.lw.data_address);
		}
		break;
	/* 4. Check for SW mem access and data miss .. add delay cycles if needed */
	case SW:
		if(!iplc_sim_trap_address(pipeline[MEM].stage.sw.data_address)){
			cycle_count = CACHE_MISS_DELAY;
			printf("DATA MISS:\t Address 0x%x\n", pipeline[MEM].stage.sw.data_address);
		}else{
			printf("DATA HIT:\t Address 0x%x\n", pipeline[MEM].stage.sw.data_address);
		}
		break;
	}
	
	/* 5. Increment pipe_cycles 1 cycle for normal processing */
	pipeline_cycles += cycle_count;

	/* 6. push stages thru MEM->WB, ALU->MEM, DECODE->ALU, FETCH->DECODE */
	pipeline[WRITEBACK] = pipeline[MEM];
	pipeline[MEM] = pipeline[ALU];
	pipeline[ALU] = pipeline[DECODE];
	pipeline[DECODE] = pipeline[FETCH];
	
	/* Reset the FETCH stage to NOP via bezero */
	bzero(&(pipeline[FETCH]), sizeof(pipeline_t));
}

/*
 * Each of the iplc_sim_process_pipeline_* functions
 * prepares the fetch stage of our pipeline with the
 * instructions sent from iplc_sim_parse_instruction
 */

void
iplc_sim_process_pipeline_rtype(byte *instruction, int dest_reg, int reg1, int reg2_or_constant)
{
	iplc_sim_push_pipeline_stage();
	
	pipeline[FETCH].itype = RTYPE;
	pipeline[FETCH].instruction_address = instruction_address;
	
	strcpy(pipeline[FETCH].stage.rtype.instruction, instruction);
	pipeline[FETCH].stage.rtype.reg1 = reg1;
	pipeline[FETCH].stage.rtype.reg2_or_constant = reg2_or_constant;
	pipeline[FETCH].stage.rtype.dest_reg = dest_reg;
}

void
iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, uint data_address)
{
	iplc_sim_push_pipeline_stage();

	pipeline[FETCH].itype = LW;
	pipeline[FETCH].instruction_address = instruction_address;

	pipeline[FETCH].stage.lw.dest_reg = dest_reg;
	pipeline[FETCH].stage.lw.base_reg = base_reg;
	pipeline[FETCH].stage.lw.data_address = data_address;
}

void
iplc_sim_process_pipeline_sw(int src_reg, int base_reg, uint data_address)
{
	iplc_sim_push_pipeline_stage();

	pipeline[FETCH].itype = SW;
	pipeline[FETCH].instruction_address = instruction_address;
	
	pipeline[FETCH].stage.sw.src_reg = src_reg;
	pipeline[FETCH].stage.sw.base_reg = base_reg;
	pipeline[FETCH].stage.sw.data_address = data_address;
}

void
iplc_sim_process_pipeline_branch(int reg1, int reg2)
{
	iplc_sim_push_pipeline_stage();

	pipeline[FETCH].itype = BRANCH;
	pipeline[FETCH].instruction_address = instruction_address;
	branch_count++;

	pipeline[FETCH].stage.branch.reg1 = reg1;
	pipeline[FETCH].stage.branch.reg2 = reg2;
}

void
iplc_sim_process_pipeline_jump(byte *instruction)
{
	iplc_sim_push_pipeline_stage();

	/* handle both jump instructions */
	pipeline[FETCH].itype = (strncmp(instruction, "jal", 3) == 0) ? JAL : JUMP;
	pipeline[FETCH].instruction_address = instruction_address;

	strcpy(pipeline[FETCH].stage.jump.instruction, instruction);
}

void
iplc_sim_process_pipeline_syscall()
{
	iplc_sim_push_pipeline_stage();

	pipeline[FETCH].itype = SYSCALL;
	pipeline[FETCH].instruction_address = instruction_address;
}

void
iplc_sim_process_pipeline_nop()
{
	iplc_sim_push_pipeline_stage();
	/*
	pipeline[FETCH] already set to NOP,
	since it's zeroed out in iplc_sim_push_pipeline_stage.
	*/
}

/* parse functions  */

/*
 * Don't touch this function.  It is for parsing the instruction stream.
 */
uint
iplc_sim_parse_reg(byte *reg_str)
{
	int i;
	// turn comma into \n
	if (reg_str[strlen(reg_str)-1] == ',')
		reg_str[strlen(reg_str)-1] = '\n';
	
	if (reg_str[0] != '$')
		return atoi(reg_str);
	else {
		// copy down over $ byteacter than return atoi
		for (i = 0; i < strlen(reg_str); i++)
			reg_str[i] = reg_str[i+1];
		
		return atoi(reg_str);
	}
}

/*
 * Don't touch this function.  It is for parsing the instruction stream.
 */
void
iplc_sim_parse_instruction(byte *buffer)
{
	int instruction_hit = 0;
	int i=0, j=0;
	int src_reg=0;
	int src_reg2=0;
	int dest_reg=0;
	byte str_src_reg[16];
	byte str_src_reg2[16];
	byte str_dest_reg[16];
	byte str_constant[16];
	
	if (sscanf(buffer, "%x %s", &instruction_address, instruction ) != 2) {
		printf("Malformed instruction \n");
		exit(-1);
	}
	
	instruction_hit = iplc_sim_trap_address( instruction_address );
	
	// if a MISS, then push current instruction thru pipeline
	if(!instruction_hit){
		// need to subtract 1, since the stage is pushed once more for actual instruction processing
		// also need to allow for a branch miss prediction during the fetch cache miss time -- by
		// counting cycles this allows for these cycles to overlap and not doubly count.
		
		printf("INST MISS:\t Address 0x%x \n", instruction_address);
		
		for (i = pipeline_cycles, j = pipeline_cycles; i < j + CACHE_MISS_DELAY - 1; i++)
			iplc_sim_push_pipeline_stage();
	}
	else
		printf("INST HIT:\t Address 0x%x \n", instruction_address);
	
	// Parse the Instruction
	
	if (strncmp( instruction, "add", 3 ) == 0 ||
		strncmp( instruction, "sll", 3 ) == 0 ||
		strncmp( instruction, "ori", 3 ) == 0) {
		if (sscanf(buffer, "%x %s %s %s %s",
				   &instruction_address,
				   instruction,
				   str_dest_reg,
				   str_src_reg,
				   str_src_reg2 ) != 5) {
			printf("Malformed RTYPE instruction (%s) at address 0x%x \n",
				   instruction, instruction_address);
			exit(-1);
		}
		
		dest_reg = iplc_sim_parse_reg(str_dest_reg);
		src_reg = iplc_sim_parse_reg(str_src_reg);
		src_reg2 = iplc_sim_parse_reg(str_src_reg2);
		
		iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
	}
	
	else if (strncmp( instruction, "lui", 3 ) == 0) {
		if (sscanf(buffer, "%x %s %s %s",
				   &instruction_address,
				   instruction,
				   str_dest_reg,
				   str_constant ) != 4 ) {
			printf("Malformed RTYPE instruction (%s) at address 0x%x \n",
				   instruction, instruction_address );
			exit(-1);
		}
		
		dest_reg = iplc_sim_parse_reg(str_dest_reg);
		src_reg = -1;
		src_reg2 = -1;
		iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
	}
	
	else if (strncmp( instruction, "lw", 2 ) == 0 ||
			 strncmp( instruction, "sw", 2 ) == 0  ) {
		if ( sscanf( buffer, "%x %s %s %s %x",
					&instruction_address,
					instruction,
					reg1,
					offsetwithreg,
					&data_address ) != 5) {
			printf("Bad instruction: %s at address %x \n", instruction, instruction_address);
			exit(-1);
		}
		
		if (strncmp(instruction, "lw", 2 ) == 0) {
			
			dest_reg = iplc_sim_parse_reg(reg1);
			
			// don't need to worry about base regs -- just insert -1 values
			iplc_sim_process_pipeline_lw(dest_reg, -1, data_address);
		}
		if (strncmp( instruction, "sw", 2 ) == 0) {
			src_reg = iplc_sim_parse_reg(reg1);
			
			// don't need to worry about base regs -- just insert -1 values
			iplc_sim_process_pipeline_sw( src_reg, -1, data_address);
		}
	}
	else if (strncmp( instruction, "beq", 3 ) == 0) {
		// don't need to worry about getting regs -- just insert -1 values
		iplc_sim_process_pipeline_branch(-1, -1);
	}
	else if (strncmp( instruction, "jal", 3 ) == 0 ||
			 strncmp( instruction, "jr", 2 ) == 0 ||
			 strncmp( instruction, "j", 1 ) == 0 ) {
		iplc_sim_process_pipeline_jump( instruction );
	}
	else if (strncmp( instruction, "jal", 3 ) == 0 ||
			 strncmp( instruction, "jr", 2 ) == 0 ||
			 strncmp( instruction, "j", 1 ) == 0 ) {
		/*
		 * Note: no need to worry about forwarding on the jump register
		 * we'll let that one go.
		 */
		iplc_sim_process_pipeline_jump(instruction);
	}
	else if ( strncmp( instruction, "syscall", 7 ) == 0) {
		iplc_sim_process_pipeline_syscall( );
	}
	else if ( strncmp( instruction, "nop", 3 ) == 0) {
		iplc_sim_process_pipeline_nop( );
	}
	else {
		printf("Do not know how to process instruction: %s at address %x \n",
			   instruction, instruction_address );
		exit(-1);
	}
}

/* MAIN Function  */
/*
int
main()
{
	byte trace_file_name[1024];
	FILE *trace_file = NULL;
	byte buffer[80];
	int index = 10;
	int blocksize = 1;
	int assoc = 1;

	printf("Please enter the tracefile: ");
	scanf("%s", trace_file_name);
	
	trace_file = fopen(trace_file_name, "r");
	
	if(!trace_file){
		printf("fopen failed for %s file\n", trace_file_name);
		exit(-1);
	}
	
	printf("Enter Cache Size (index), Blocksize and Level of Assoc \n");
	scanf( "%d %d %d", &index, &blocksize, &assoc );
	
	printf("Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): ");
	scanf("%d", &branch_predict_taken );
	
	iplc_sim_init(index, blocksize, assoc);
	
	while(fgets(buffer, 80, trace_file) != NULL){
		iplc_sim_parse_instruction(buffer);
		if (dump_pipeline)
			iplc_sim_dump_pipeline();
	}
	
	iplc_sim_finalize();
	return 0;
}
*/

int
main(int argc, char **argv)
{
	byte *trace_file_name = "instruction-trace.txt";
	FILE *trace_file = NULL;
	byte buffer[80];
	int index = 10;
	int blocksize = 1;
	int assoc = 1;
	/*
	printf("Please enter the tracefile: ");
	scanf("%s", trace_file_name);
	*/
	trace_file = fopen(trace_file_name, "r");
	
	if(!trace_file){
		printf("fopen failed for %s file\n", trace_file_name);
		exit(-1);
	}
	/*
	printf("Enter Cache Size (index), Blocksize and Level of Assoc \n");
	scanf( "%d %d %d", &index, &blocksize, &assoc );
	
	printf("Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): ");
	scanf("%d", &branch_predict_taken );
	*/
	index = atoi(argv[1]);
	blocksize = atoi(argv[2]);
	assoc = atoi(argv[3]);
	branch_predict_taken = atoi(argv[4]);
	iplc_sim_init(index, blocksize, assoc);
	
	while(fgets(buffer, 80, trace_file) != NULL){
		iplc_sim_parse_instruction(buffer);
		if (dump_pipeline)
			iplc_sim_dump_pipeline();
	}
	
	iplc_sim_finalize();
	return 0;
}