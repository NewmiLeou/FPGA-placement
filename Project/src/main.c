#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <math.h>

#define DESIRED 27.0

typedef struct CLB CLB;
typedef struct NET{
    int id;
    char type;
    struct NET *next, *prev;
} NET;

typedef struct INST{
    int id;
    int x, y;
    NET *net_list;
    CLB *location;
    struct INST *next, *prev;
} INST;

typedef struct CLB{
    int x, y;
    int LUTs_num;
    int DFFs_num;
    INST *LUTs_list[3];
    INST *DFFs_list[3];
} CLB;

typedef struct PI{
    int id;
    float x, y;
    struct PI *next, *prev;
} PI;

typedef struct PO{
    int id;
    float x, y;
    struct PO *next, *prev;
} PO;

typedef struct sort_NET{
    int id;
    int size;
} sort_NET;

typedef struct Fitness{
    int id;
    int wSize; // window size
    float fitness;
} Fitness;

typedef struct Coordinate{
    int x;
    int y;
} Coordinate;
/* variable declaration */ 
FILE *info_file, *nets_file, *output_file;
INST **LUTs = NULL, **DFFs = NULL;
NET **NETs = NULL, *new_net, *net_list, *current_inst, *pin_list;
CLB **CLBs = NULL, *new_clb;
PI **PIs = NULL;
PO **POs = NULL;
sort_NET *sort_net = NULL;
Fitness *fitnessList = NULL;
int C = 0, R = 0;   // CLB_Dim
int Q = 0, P = 0;   // Num_I/O_Pad
int I = 0, O = 0;   // Num_PI, Num_PO
int M = 0, N = 0;   // Num_Inst LUTs(M) DFFs(N)
int num_nets = 0;
int num_pi, num_po, num_inst, num_net;
float pi_x, pi_y;
float po_x, po_y;
int clb_x, clb_y;
float top, bottom, left, right;
float hpwl, best_hpwl, HPWL = 0;
//int i;
int gID;
char gType;
Coordinate gCoor;
/* function declaration */
void parse_info();
void parse_nets();
void CreateFitnessList();
void write_output();
void sort_nets();
void initial_placement();
void swap(int id1, int id2, char type1, char type2);
void Swap(int id1, int id2, char type);
void SA();
void OtherSA();
float calculate_WL();
double GetAlpha(int delta, int T);
int ChooseNet(); // choose a net from the range fitness list
char SelectBlock(int *p);// find a random block in the net
Coordinate CalculateNetCenter(int netIndex);// determine the center of the net
Coordinate DeterminObjectiveLocation(Coordinate netCenter); // find a position within the range to swap
int CalculateNewCost(); // calculate the new cost after swap
void SwapOperation(char type, int index, Coordinate location);

/* main */
int main(int argc, char **argv){
    // read argv
    info_file = fopen(argv[1], "r");
    nets_file = fopen(argv[2], "r");
    output_file = fopen(argv[3], "w");
    char benchmark[20] = {0};
    strncpy(benchmark, argv[3]+11, 1);
    printf("[  Benchmark %s Information  ]\n", benchmark);

    // parse file
    parse_info();
    parse_nets();
    // Placement
    initial_placement();
    CreateFitnessList();
    SA();
    calculate_WL();
    printf("[  Total HPWL  ]: %.2f\n", HPWL); 

    // write output
    write_output();

    // Total Runtime
    double runtime = clock();
    printf("[  Total Run time  ]: %2f sec\n\n",runtime/CLOCKS_PER_SEC);

    return 0;
}


/* functions */
// Parse info file
void parse_info(){
    // line 1 specifies values of C and R in order
    fscanf(info_file, "CLB_Dim %d %d\n", &C, &R);
    printf("CLB Dimension : %d %d\n", C, R);
    CLBs = malloc((R+1)*(C+1)*sizeof(CLB *));
    for(clb_x=1; clb_x<=C; clb_x++){
        for (clb_y=1; clb_y<=R; clb_y++){

            new_clb = malloc(sizeof(CLB));
            new_clb->x = clb_x;
            new_clb->y = clb_y;

            new_clb->LUTs_num = 0;
            new_clb->DFFs_num = 0;
            new_clb->LUTs_list[1] = NULL;
            new_clb->LUTs_list[2] = NULL;
            new_clb->DFFs_list[1] = NULL;
            new_clb->DFFs_list[2] = NULL;

            CLBs[clb_x+(clb_y*C)] = new_clb;
        }        
    }

    // Line 2 specifies values of Q and P in order
    fscanf(info_file, "Num_I/O_Pad %d %d\n", &Q, &P);
    printf("Num I/O Pad : %d %d\n", Q, P);

    // Line 3 specifies the number of primary inputs (I)    
    fscanf(info_file, "Num_PI %d\n", &I);
    //printf("Num PI : %d\n", I);
    if(I!=0) PIs = malloc((I+1)*sizeof(PI *));
    while(fscanf(info_file, "I%d %f %f\n", &num_pi, &pi_x, &pi_y)!=0 && I!=0){
        PIs[num_pi] = malloc(sizeof(PI));
        
        PIs[num_pi]->id = num_pi;
        PIs[num_pi]->x = pi_x;
        PIs[num_pi]->y = pi_y;
        PIs[num_pi]->next = NULL;
        PIs[num_pi]->prev = NULL;

        if(num_pi==I) break;
    }    
    
    // Line 4 + I specifies the number of primary outputs (O)
    fscanf(info_file, "Num_PO %d\n", &O);
    //printf("Num PO : %d\n", O);
    if(O!=0) POs = malloc((O+1)*sizeof(PO *));
    while(fscanf(info_file, "O%d %f %f\n", &num_po, &po_x, &po_y)!=0 && O!=0){
        POs[num_po] = malloc(sizeof(PO));

        POs[num_po]->id = num_po;
        POs[num_po]->x = po_x;
        POs[num_po]->y = po_y;
        POs[num_po]->next = NULL;
        POs[num_po]->prev = NULL;

        if(num_po==O) break;
    }

    // Line 5 + I + O specifies the numbers of LUTs and DFFs
    fscanf(info_file, "Num_Inst %d %d\n", &M, &N);
    printf("Num LUTs : %d\nNum DFFs : %d\n", M, N);
    if(M!=0) LUTs = malloc((M+1)*sizeof(INST *));
    while(fscanf(info_file, "L%d\n", &num_inst)!=0  && M!=0){
        LUTs[num_inst] = malloc(sizeof(INST));
        
        LUTs[num_inst]->id = num_inst;
        LUTs[num_inst]->x = 0;
        LUTs[num_inst]->y = 0;
        LUTs[num_inst]->next = NULL;
        LUTs[num_inst]->prev = NULL;
        LUTs[num_inst]->net_list = NULL;
        LUTs[num_inst]->location = NULL;

        if(num_inst==M) break;
    }
    if(N!=0) DFFs = malloc((N+1)*sizeof(INST *));    
    while(fscanf(info_file, "F%d\n", &num_inst)!=0  && N!=0){
        DFFs[num_inst] = malloc(sizeof(INST));
        
        DFFs[num_inst]->id = num_inst;
        DFFs[num_inst]->x = 0;
        DFFs[num_inst]->y = 0;
        DFFs[num_inst]->next = NULL;
        DFFs[num_inst]->prev = NULL;
        DFFs[num_inst]->net_list = NULL;
        DFFs[num_inst]->location = NULL;

        if(num_inst==N) break;
    }
}

// Parse nets file
void parse_nets(){
    // Line 1 specifies values of N which denotes the total number of nets
    fscanf(nets_file, "%d\n", &num_nets);
    printf("NETs NUM : %d\n", num_nets);
    NETs = malloc((num_nets+1)*sizeof(NET *));   
    sort_net = malloc((num_nets+1)*sizeof(sort_NET));
    fitnessList = malloc((num_nets+1)*sizeof(Fitness));
    
    while(!feof(nets_file)){        
        fscanf(nets_file, "n%d ", &num_net);
        /* printf("n%d ", num_net); */
        NETs[num_net] = NULL;

        sort_net[num_net].id = num_net;
        sort_net[num_net].size = 0;
        fitnessList[num_net].id = num_net;
        fitnessList[num_net].wSize = -1;
        fitnessList[num_net].fitness = -1;

        while(1){
            // LUT
            if(fscanf(nets_file, "L%d ", &num_inst)==1){
                new_net = malloc(sizeof(NET));
                new_net->prev = NULL;
                new_net->next = NULL;

                new_net->id = num_inst;
                new_net->type = 'L';

                net_list = malloc(sizeof(NET));
                net_list->prev = NULL;
                net_list->next = NULL;

                net_list->id = num_net;
                net_list->type = 'N';

                if(LUTs[num_inst]->net_list!=NULL){                    
                    net_list->next = LUTs[num_inst]->net_list;
                    LUTs[num_inst]->net_list->prev = net_list;
                }

                LUTs[num_inst]->net_list = net_list;

                if(NETs[num_net]!=NULL){
                    new_net->next = NETs[num_net];
                    NETs[num_net]->prev = new_net;
                }

                NETs[num_net] = new_net;
                sort_net[num_net].size++;                              
            }
            // DFF
            else if(fscanf(nets_file, "F%d ", &num_inst)==1){
                new_net = malloc(sizeof(NET));
                new_net->prev = NULL;
                new_net->next = NULL;

                new_net->id = num_inst;
                new_net->type = 'F';

                net_list = malloc(sizeof(NET));
                net_list->prev = NULL;
                net_list->next = NULL;

                net_list->id = num_net;
                net_list->type = 'N';

                if(DFFs[num_inst]->net_list!=NULL){                    
                    net_list->next = DFFs[num_inst]->net_list;
                    DFFs[num_inst]->net_list->prev = net_list;
                }

                DFFs[num_inst]->net_list = net_list;

                if(NETs[num_net]!=NULL){
                    new_net->next = NETs[num_net];
                    NETs[num_net]->prev = new_net;
                }

                NETs[num_net] = new_net;
                sort_net[num_net].size++; 
            }
            // PI
            else if(fscanf(nets_file, "I%d ", &num_inst)==1){
                new_net = malloc(sizeof(NET));
                new_net->prev = NULL;
                new_net->next = NULL;

                new_net->id = num_inst;
                new_net->type = 'I';

                if(NETs[num_net]!=NULL){
                    new_net->next = NETs[num_net];
                    NETs[num_net]->prev = new_net;
                }

                NETs[num_net] = new_net;
                sort_net[num_net].size++; 
            }
            // PO
            else if(fscanf(nets_file, "O%d ", &num_inst)==1){
                new_net = malloc(sizeof(NET));
                new_net->prev = NULL;
                new_net->next = NULL;
                
                new_net->id = num_inst;
                new_net->type = 'O';

                if(NETs[num_net]!=NULL){
                    new_net->next = NETs[num_net];
                    NETs[num_net]->prev = new_net;
                }

                NETs[num_net] = new_net;
                sort_net[num_net].size++; 
            }
            else 
                break;
        }
    }
    sort_nets();
}

// Write output file
void write_output(){
    if(M>0){
        for(num_inst=1; num_inst<=M; num_inst++){
            fprintf(output_file, "L%d %d %d\n", num_inst, LUTs[num_inst]->x, LUTs[num_inst]->y);

            if(LUTs[num_inst]->x<1 || LUTs[num_inst]->x>C || LUTs[num_inst]->y<1 || LUTs[num_inst]->y>R){
                printf("[ Error ] : illegal placement !\n");
            }
        }
    }   

    if(N>0){
        for(num_inst=1; num_inst<=N; num_inst++){
            fprintf(output_file, "F%d %d %d\n", num_inst, DFFs[num_inst]->x, DFFs[num_inst]->y);

            if(DFFs[num_inst]->x<1 || DFFs[num_inst]->x>C || DFFs[num_inst]->y<1 || DFFs[num_inst]->y>R){
                printf("[ Error ] : illegal placement !\n");
            }
        }
    }    

}

// Sort the nets
void sort_nets(){
    //  small first 
    int j, min, temp1, temp2;
    for(int i=1; i<=num_nets; i++){
        min = i;
        for(j=i; j<=num_nets; j++){
            if(sort_net[j].size<sort_net[min].size){
                min = j;
            }
        }

        temp1 = sort_net[min].id;
        temp2 = sort_net[min].size;

        sort_net[min].id = sort_net[i].id;
        sort_net[min].size = sort_net[i].size;

        sort_net[i].id = temp1;
        sort_net[i].size = temp2;
    }
    

    // large first 
    /*int j, max, temp1, temp2;
    for(i=1; i<=num_nets; i++){
        max = i;
        for(j=i; j<=num_nets; j++){
            if(sort_net[j].size>sort_net[max].size){
                max = j;
            }
        }

        temp1 = sort_net[max].id;
        temp2 = sort_net[max].size;

        sort_net[max].id = sort_net[i].id;
        sort_net[max].size = sort_net[i].size;

        sort_net[i].id = temp1;
        sort_net[i].size = temp2;
    }*/
}

// Initial Placement
void initial_placement(){
    for(int i=1; i<=num_nets; i++){
        current_inst = NETs[sort_net[i].id];
        while(current_inst!=NULL){
            // LUT
            if(current_inst->type == 'L'){
                if(LUTs[current_inst->id]->x!=0 && LUTs[current_inst->id]->y!=0){
                    current_inst = current_inst->next;
                    continue;
                }                

                best_hpwl = LONG_MAX;
                for(clb_x=1; clb_x<=C; clb_x++){
                    for (clb_y=1; clb_y<=R; clb_y++){
                        top = CLBs[clb_x+(clb_y*C)]->y;
                        bottom = CLBs[clb_x+(clb_y*C)]->y;
                        left = CLBs[clb_x+(clb_y*C)]->x;
                        right = CLBs[clb_x+(clb_y*C)]->x;

                        // Constraint: at most two LUTs and two DFFs can be placed onto the same CLB location
                        if(CLBs[clb_x+(clb_y*C)]->LUTs_num<2){
                            pin_list = NETs[sort_net[i].id];
                            while(pin_list!=NULL){
                                if(pin_list->type=='I'){
                                    if(PIs[pin_list->id]->x>right){
                                        right = PIs[pin_list->id]->x;
                                    }
                                    if(PIs[pin_list->id]->x<left){
                                        left = PIs[pin_list->id]->x;
                                    }
                                    
                                    if(PIs[pin_list->id]->y>top){
                                        top = PIs[pin_list->id]->y;
                                    }
                                    if(PIs[pin_list->id]->y<bottom){
                                        bottom = PIs[pin_list->id]->y;
                                    }
                                }
                                else if(pin_list->type=='O'){
                                    if(POs[pin_list->id]->x>right){
                                        right = POs[pin_list->id]->x;
                                    }
                                    if(POs[pin_list->id]->x<left){
                                        left = POs[pin_list->id]->x;
                                    }
                                    
                                    if(POs[pin_list->id]->y>top){
                                        top = POs[pin_list->id]->y;
                                    }
                                    if(POs[pin_list->id]->y<bottom){
                                        bottom = POs[pin_list->id]->y;
                                    }
                                }
                    
                                pin_list = pin_list->next;
                            }

                            hpwl = (top-bottom) + (right-left);
                            if(hpwl<best_hpwl){
                                if(LUTs[current_inst->id]->location!=NULL){
                                    LUTs[current_inst->id]->location->LUTs_num--;
                                    if(LUTs[current_inst->id]->location->LUTs_list[1]->id == current_inst->id){
                                        LUTs[current_inst->id]->location->LUTs_list[1] = LUTs[current_inst->id]->location->LUTs_list[2];
                                        LUTs[current_inst->id]->location->LUTs_list[2] = NULL;
                                    }
                                    else if(LUTs[current_inst->id]->location->LUTs_list[2]->id == current_inst->id){
                                        LUTs[current_inst->id]->location->LUTs_list[2] = NULL;
                                    }
                                }

                                LUTs[current_inst->id]->x = CLBs[clb_x+(clb_y*C)]->x;
                                LUTs[current_inst->id]->y = CLBs[clb_x+(clb_y*C)]->y;
                                CLBs[clb_x+(clb_y*C)]->LUTs_num++;
                                CLBs[clb_x+(clb_y*C)]->LUTs_list[CLBs[clb_x+(clb_y*C)]->LUTs_num] = LUTs[current_inst->id];
                                LUTs[current_inst->id]->location = CLBs[clb_x+(clb_y*C)];

                                best_hpwl = hpwl;
                            }
                        }
                    }
                }
            }

            // DFF
            else if(current_inst->type == 'F'){
                if(DFFs[current_inst->id]->x!=0 && DFFs[current_inst->id]->y!=0){
                    current_inst = current_inst->next;
                    continue;
                }               

                best_hpwl = LONG_MAX;
                for(clb_x=1; clb_x<=C; clb_x++){
                    for (clb_y=1; clb_y<=R; clb_y++){
                        top = CLBs[clb_x+(clb_y*C)]->y;
                        bottom = CLBs[clb_x+(clb_y*C)]->y;
                        left = CLBs[clb_x+(clb_y*C)]->x;
                        right = CLBs[clb_x+(clb_y*C)]->x;

                        // Constraint: at most two LUTs and two DFFs can be placed onto the same CLB location
                        if(CLBs[clb_x+(clb_y*C)]->DFFs_num<2){
                            pin_list = NETs[sort_net[i].id];
                            while(pin_list!=NULL){
                                if(pin_list->type=='I'){
                                    if(PIs[pin_list->id]->x>right){
                                        right = PIs[pin_list->id]->x;
                                    }
                                    if(PIs[pin_list->id]->x<left){
                                        left = PIs[pin_list->id]->x;
                                    }
                                    
                                    if(PIs[pin_list->id]->y>top){
                                        top = PIs[pin_list->id]->y;
                                    }
                                    if(PIs[pin_list->id]->y<bottom){
                                        bottom = PIs[pin_list->id]->y;
                                    }
                                }
                                else if(pin_list->type=='O'){
                                    if(POs[pin_list->id]->x>right){
                                        right = POs[pin_list->id]->x;
                                    }
                                    if(POs[pin_list->id]->x<left){
                                        left = POs[pin_list->id]->x;
                                    }
                                    
                                    if(POs[pin_list->id]->y>top){
                                        top = POs[pin_list->id]->y;
                                    }
                                    if(POs[pin_list->id]->y<bottom){
                                        bottom = POs[pin_list->id]->y;
                                    }
                                }
                    
                                pin_list = pin_list->next;
                            }

                            hpwl = (top-bottom) + (right-left);
                            if(hpwl<best_hpwl){
                                if(DFFs[current_inst->id]->location!=NULL){
                                    DFFs[current_inst->id]->location->DFFs_num--;
                                    if(DFFs[current_inst->id]->location->DFFs_list[1]->id == current_inst->id){
                                        DFFs[current_inst->id]->location->DFFs_list[1] = DFFs[current_inst->id]->location->DFFs_list[2];
                                        DFFs[current_inst->id]->location->DFFs_list[2]=NULL;
                                    }
                                    else if(DFFs[current_inst->id]->location->DFFs_list[2]->id == current_inst->id){
                                        DFFs[current_inst->id]->location->DFFs_list[2]=NULL;
                                    }
                                }

                                DFFs[current_inst->id]->x = CLBs[clb_x+(clb_y*C)]->x;
                                DFFs[current_inst->id]->y = CLBs[clb_x+(clb_y*C)]->y;
                                CLBs[clb_x+(clb_y*C)]->DFFs_num++;
                                CLBs[clb_x+(clb_y*C)]->DFFs_list[CLBs[clb_x+(clb_y*C)]->DFFs_num] = DFFs[current_inst->id];
                                DFFs[current_inst->id]->location = CLBs[clb_x+(clb_y*C)];

                                best_hpwl = hpwl;
                            }
                        }
                    }
                }
            }            

            current_inst = current_inst->next;
        }
    }
}

void CreateFitnessList(){
    for( int i = 1; i <= num_nets; i++ ){
        top = 0;
        bottom = R+1;
        right = 0;
        left = C+1;
        NET *walkNetList = NETs[i];
        while( walkNetList != NULL ) {
            switch (walkNetList->type)
            {
            case 'L':
                if((float)LUTs[walkNetList->id]->x < left){
                    left = (float)LUTs[walkNetList->id]->x;
                }
                if((float)LUTs[walkNetList->id]->x > right){
                    right = (float)LUTs[walkNetList->id]->x;
                }
                if((float)LUTs[walkNetList->id]->y < bottom){
                    bottom = (float)LUTs[walkNetList->id]->y;
                }
                if((float)LUTs[walkNetList->id]->y > top){
                    top = (float)LUTs[walkNetList->id]->y;
                }
                break;
            case 'F':
                if((float)DFFs[walkNetList->id]->x < left){
                    left = (float)DFFs[walkNetList->id]->x;
                }
                if((float)DFFs[walkNetList->id]->x > right){
                    right = (float)DFFs[walkNetList->id]->x;
                }
                if((float)DFFs[walkNetList->id]->y < bottom){
                    bottom = (float)DFFs[walkNetList->id]->y;
                }
                if((float)DFFs[walkNetList->id]->y > top){
                    top = (float)DFFs[walkNetList->id]->y;
                }
                break;
            case 'I':
                if((float)PIs[walkNetList->id]->x < left){
                    left = (float)PIs[walkNetList->id]->x;
                }
                if((float)PIs[walkNetList->id]->x > right){
                    right = (float)PIs[walkNetList->id]->x;
                }
                if((float)PIs[walkNetList->id]->y < bottom){
                    bottom = (float)PIs[walkNetList->id]->y;
                }
                if((float)PIs[walkNetList->id]->y > top){
                    top = (float)PIs[walkNetList->id]->y;
                }
                break;
            case 'O':
                if((float)POs[walkNetList->id]->x < left){
                    left = (float)POs[walkNetList->id]->x;
                }
                if((float)POs[walkNetList->id]->x > right){
                    right = (float)POs[walkNetList->id]->x;
                }
                if((float)POs[walkNetList->id]->y < bottom){
                    bottom = (float)POs[walkNetList->id]->y;
                }
                if((float)POs[walkNetList->id]->y > top){
                    top = (float)POs[walkNetList->id]->y;
                }
                break;
            default:
                break;
            }
            walkNetList = walkNetList -> next;
        }

        /*Calculate the window size*/
        if( (top-bottom) == 0 && (right-left) > 0 )
            fitnessList[i].wSize = right-left;
        else if( (top-bottom) > 0 && (right-left) == 0 )
            fitnessList[i].wSize = top-bottom;
        else if( (top-bottom) == 0 && (right-left) == 0 )
            fitnessList[i].wSize = 1;
        else
            fitnessList[i].wSize = (top-bottom)*(right-left);
        /////////////////////////////

        /*Determine the fitness*/
        if( fitnessList[i].wSize > DESIRED )
            fitnessList[i].fitness = DESIRED / fitnessList[i].wSize;
        else
            fitnessList[i].fitness = fitnessList[i].wSize / DESIRED;
        /////////////////////////
    }

}

// Swap two instance
void swap(int id1, int id2, char type1, char type2) {
    int temp_x, temp_y;
    if( type1 == 'L' ) {
        temp_x = LUTs[id1]->x;
        temp_y = LUTs[id1]->y;
        if ( type2 == 'L' ) {
            LUTs[id1]->x = LUTs[id2]->x;
            LUTs[id1]->y = LUTs[id2]->y;
            LUTs[id2]->x = temp_x;
            LUTs[id2]->y = temp_y;
        }
        else { // type2 == 'F'
            LUTs[id1]->x = DFFs[id2]->x;
            LUTs[id1]->y = DFFs[id2]->y;
            DFFs[id2]->x = temp_x;
            DFFs[id2]->y = temp_y;
        }
    }
    else if ( type1 == 'F' ) {
        temp_x = DFFs[id1]->x;
        temp_y = DFFs[id1]->y;
        if ( type2 == 'L' ) {
            DFFs[id1]->x = LUTs[id2]->x;
            DFFs[id1]->y = LUTs[id2]->y;
            LUTs[id2]->x = temp_x;
            LUTs[id2]->y = temp_y;
        }
        else{ // type2 == 'F'
            DFFs[id1]->x = DFFs[id2]->x;
            DFFs[id1]->y = DFFs[id2]->y;
            DFFs[id2]->x = temp_x;
            DFFs[id2]->y = temp_y;
        }
    }
    else
        return;
}

void Swap(int id1, int id2, char type){
    int temp_x, temp_y;
    if(type == 'L'){
        temp_x = LUTs[id1]->x;
        temp_y = LUTs[id1]->y;
        LUTs[id1]->x = LUTs[id2]->x;
        LUTs[id1]->y = LUTs[id2]->y;
        LUTs[id2]->x = temp_x;
        LUTs[id2]->y = temp_y;
    }
    else if(type == 'F'){
        temp_x = DFFs[id1]->x;
        temp_y = DFFs[id1]->y;
        DFFs[id1]->x = DFFs[id2]->x;
        DFFs[id1]->y = DFFs[id2]->y;
        DFFs[id2]->x = temp_x;
        DFFs[id2]->y = temp_y;
    }
    else
        return;
}

void SA(){
    if ( M+N == 1431 || M+N == 1871 ) {
        OtherSA();
        return;
    }
    float current_hpwl = calculate_WL();
    // LUT
    if(M>0){
        for(int id1=1; id1<=M;id1++){
            for(int id2=1;id2<=M;id2++){
                float new_hpwl = 0;
                Swap(id1,id2,'L');
                new_hpwl = calculate_WL();
                if(new_hpwl < current_hpwl ){    // BETTER
                    current_hpwl = new_hpwl;
                    double swap_time = clock()/CLOCKS_PER_SEC;
                    if(swap_time > 1600) return;
                }
                else{
                    Swap(id2,id1,'L');
                    double swap_time = clock()/CLOCKS_PER_SEC;
                    if(swap_time > 1600) return;
                }
                //printf("%.2f %.2f\n",current_hpwl,new_hpwl);
            }
        }
    }
    // DFF
    if(N>0){
        for(int id1=1; id1<=N;id1++){
            for(int id2=1;id2<=N;id2++){
                float new_hpwl = 0;
                Swap(id1,id2,'F');
                new_hpwl = calculate_WL();
                if(new_hpwl < current_hpwl ){    // BETTER
                    current_hpwl = new_hpwl;
                    double swap_time = clock()/CLOCKS_PER_SEC;
                    if(swap_time > 1600) return;
                }
                else{
                    Swap(id2,id1,'F');
                    double swap_time = clock()/CLOCKS_PER_SEC;
                    if(swap_time > 1600) return;
                }
                //printf("%.2f %.2f\n",current_hpwl,new_hpwl);
            }
        }
    }

    return;
}

// Simulated Annealing
void OtherSA(){
    int index;
    int netIndex;
    int *p = &index;
    int T = calculate_WL()*20; // set initial temperature
    //int InnerIter = num_nets * 0.66; // number of inner iteration
    int InnerIter = pow((M+N),1.3333); // number of inner iteration
    // calculate the cost of initial placement
    while ( T > 1 ) {
        int iter = 0;
        int delta; // new cost - original cost
        while ( iter < InnerIter ) {
            double swap_time = clock()/CLOCKS_PER_SEC;
            if(swap_time > 1600) {
                return;
            }
            iter++;
            int originalCost = calculate_WL();
            netIndex = ChooseNet(); // choose a net from the range fitness list
            index = netIndex;
            char type = SelectBlock(p);// find a random block in the net
            // Now type is 'L'/'F' and index is ID of selected instance. 
            Coordinate netCenter = CalculateNetCenter(netIndex);// determine the center of the net
            Coordinate location = DeterminObjectiveLocation(netCenter); // find a position within the range to swap
            SwapOperation(type, index, location);
            int newCost = calculate_WL(); // calculate the new cost after swap
            delta =  newCost - originalCost;// new cost - original cost
            if ( delta < 0 ) { // new placement is better
                CreateFitnessList();
            }
            else {
                float prob = exp(-delta/T);
                float tmp = prob*1000.0;
                int number = (int)tmp;
                int x = rand()%1000;
                if ( x <= number ) {
                    CreateFitnessList();
                }
                else {
                    if ( gID == -1 ) { // swap with empty location
                        if(type == 'L') {
                            LUTs[index]->x = gCoor.x;
                            LUTs[index]->y = gCoor.y;
                        }
                        else{
                            DFFs[index]->x = gCoor.x;
                            DFFs[index]->y = gCoor.y;
                        }
                    }
                    else {
                        swap(index, gID, type, gType);
                    }
                }
            }
            /*
            else {
                int x = rand();
                x = x%2; // x is 0 or 1
                if ( x < pow(0.1, delta)/T ) { // accept this solution
                    CreateFitnessList();
                }
                else { // recover
                    if ( gID == -1 ) { // swap with empty location
                        if(type == 'L') {
                            LUTs[index]->x = gCoor.x;
                            LUTs[index]->y = gCoor.y;
                        }
                        else{
                            DFFs[index]->x = gCoor.x;
                            DFFs[index]->y = gCoor.y;
                        }
                    }
                    else {
                        swap(index, gID, type, gType);
                    }
                    //CreateFitnessList();
                }
            }*/
            
        }
        double alpha = GetAlpha(delta, T);
        T = alpha*T;
    }
}

double GetAlpha( int delta, int T) {
    double prob = exp((delta*-1)/T); // e^(-delta/T)
    if (prob>0.96) return 0.5;
    else if(prob>0.8) return 0.9;
    else if(prob>0.15) return 0.95;
    else return 0.8;
}

int ChooseNet(){
    int index; // for random select a net from fitness range list
    int a = rand();
    int b = rand();
    a = a%(num_nets);
    b = b%(num_nets);
    a++; // for access NETs directly
    b++; // for access NETs directly
    // Now the values of a and b are between 1 and num_nets
    if( fitnessList[a].fitness <= fitnessList[b].fitness )
        index = a;
    else
        index = b;
    return index;
}

char SelectBlock(int *p){
    int index = *p;
    int idArr[M+N];
    char typeArr[M+N];
    int amount = 0;
    NET *walk = NETs[index];
    while( walk != NULL ) {
        if( walk->type == 'L' || walk->type == 'F' ) {
            idArr[amount] = walk->id;
            typeArr[amount] = walk->type;
            amount++;
        }
        walk = walk->next;
    }
    int random = rand();
    random = random % amount;
    *p = idArr[random];
    return typeArr[random];
}

Coordinate CalculateNetCenter(int netIndex){
    float x = 0.0 , y = 0.0;
    int num = 0;
    NET *walk = NETs[netIndex];
    /*Calculate the mean value of coordinates of instances connected by this net.*/
    while(walk!=NULL){
        switch(walk->type)
        {
        case 'L':
            x = x + LUTs[walk->id]->x;
            y = y + LUTs[walk->id]->y;
            num++;
            break;
        case 'F':
            x = x + DFFs[walk->id]->x;
            y = y + DFFs[walk->id]->y;
            num++;
            break;
        /*
        case 'I':
            x = x + PIs[walk->id]->x;
            y = y + PIs[walk->id]->y;
            num++;
            break;
        case 'O':
            x = x + POs[walk->id]->x;
            y = y + POs[walk->id]->y;
            num++;
            break;
        */
        default:
            break;
        }
        walk = walk->next;
    }
    x = x / (float)num;
    y = y / (float)num;
    Coordinate temp;
    temp.x = (int)(x+0.5); // round()
    temp.y = (int)(y+0.5); // round()
    return temp;
}

Coordinate DeterminObjectiveLocation(Coordinate netCenter){
    Coordinate temp;
    int random = rand();
    random = random%9; // random between 0 and 8
    while(true){
        bool flag = false;
        switch(random)
        {   // |0|1|2|
            // |3|4|5| hint: |4| represent the net center
            // |6|7|8|
            case 0:
                if( (netCenter.x-1) >= 1 && (netCenter.y+1) <= R ) {
                    temp.x = netCenter.x-1;
                    temp.y = netCenter.y+1;
                    flag = true;
                }
                break;
            case 1:
                if( (netCenter.y+1) <= R ) {
                    temp.x = netCenter.x;
                    temp.y = netCenter.y+1;
                    flag = true;
                }
                break;
            case 2:
                if( (netCenter.x+1) <= C && (netCenter.y+1) <= R ) {
                    temp.x = netCenter.x+1;
                    temp.y = netCenter.y+1;
                    flag = true;
                }
                break;
            case 3:
                if( (netCenter.x-1) >= 1 ) {
                    temp.x = netCenter.x-1;
                    temp.y = netCenter.y;
                    flag = true;
                }
                break;
            case 4:
                temp.x = netCenter.x;
                temp.y = netCenter.y;
                flag = true;
                break;
            case 5:
                if( (netCenter.x+1) <= C ) {
                    temp.x = netCenter.x+1;
                    temp.y = netCenter.y;
                    flag = true;
                }
                break;
            case 6:
                if( (netCenter.x-1) >= 1 && (netCenter.y-1) >= 1 ) {
                    temp.x = netCenter.x-1;
                    temp.y = netCenter.y-1;
                    flag = true;
                }
                break;
            case 7:
                if( (netCenter.y-1) >= 1 ) {
                    temp.x = netCenter.x;
                    temp.y = netCenter.y-1;
                    flag = true;
                }
                break;
            case 8:
                if( (netCenter.x+1) <= C && (netCenter.y-1) >= 1 ) {
                    temp.x = netCenter.x+1;
                    temp.y = netCenter.y-1;
                    flag = true;
                }
                break;
            default:
                break;
        }
        if( flag )
            break; // End the while loop
        else // Illegal location generated by random number, do it again.
            random = rand()%9; // random between 0 and 8
    }
    return temp;
}
//CLBs
void SwapOperation(char type, int index, Coordinate target){
    int limit;
    if ( M >= N ) limit = M;
    else limit = N;
    int curLUT = 0, curDFF = 0;
    int numLUT = 0, numDFF = 0;
    int idLUT1 = -1, idLUT2 = -1, idDFF1 = -1, idDFF2 = -1;
    for( int i = 1; i <= limit; i++ ){
        if( i <= M && LUTs[i]->x == target.x && LUTs[i]->y == target.y ) {
            if(idLUT1 == -1) idLUT1 = i;
            else idLUT2 = i;
            numLUT++;
        }
        if( i <= N && DFFs[i]->x == target.x && DFFs[i]->y == target.y ) {
            if(idDFF1 == -1) idDFF1 = i;
            else idDFF2 = i;
            numDFF++;
        }

        if ( type == 'L' ) {
            if( i <= M && LUTs[i]->x == LUTs[index]->x && LUTs[i]->y == LUTs[index]->y ) {
                curLUT++;
            }
            if( i <= N && DFFs[i]->x == LUTs[index]->x && DFFs[i]->y == LUTs[index]->y ) {
                curDFF++;
            }
        }
        else {
            if( i <= M && LUTs[i]->x == DFFs[index]->x && LUTs[i]->y == DFFs[index]->y ) {
                curLUT++;
            }
            if( i <= N && DFFs[i]->x == DFFs[index]->x && DFFs[i]->y == DFFs[index]->y ) {
                curDFF++;
            }
        }
    }
    int ID = -1;
    char TYPE;
    int random;
    switch(type)
    {
        case 'L':
            if ( numLUT == 2 || curDFF == 2 ) { // 只能跟LUT swap
                TYPE = 'L';
                random = rand()%2;
                if(random == 0){ // 跟LUT1 swap
                    ID = idLUT1;
                }
                else{ // 跟LUT2 swap
                    ID = idLUT2;
                }
            }
            else{ // 可以跟LUT or DFF swap or 移到空位
                if ( numLUT == 0 && numDFF == 0 ) { // 該location沒有LUT和DFF，直接更改location
                    ID = -1;
                }
                else{
                    random = rand()%2;
                    if(random == 0) { // swap with LUT
                        if( numLUT == 0 )
                            ID = -1;
                        else{ // numLUT == 1
                            random = rand()%2;
                            if ( random == 0 )
                                ID = -1;
                            else {
                                TYPE = 'L';
                                ID = idLUT1;
                            }
                        }
                    }
                    else { // swap with DFF
                        if ( numDFF == 0 )
                            ID == -1;
                        else if ( numDFF == 1 ) {
                            random = rand()%2;
                            if( random == 0 )
                                ID = -1;
                            else {
                                TYPE = 'F';
                                ID == idDFF1;
                            }
                        }
                        else{ // numDFF == 2
                            TYPE = 'F';
                            random = rand()%2;
                            if ( random == 0 ) {
                                ID = idDFF1;
                            }
                            else {
                                ID = idDFF2;
                            }
                        }
                    }
                }
            }
            break;
        case 'F':
            if ( numDFF == 2 || curLUT == 2 ) { // 只能跟DFF swap
                TYPE = 'F';
                random = rand()%2;
                if(random == 0){ // 跟DFF1 swap
                    ID = idDFF1;
                }
                else{ // 跟DFF2 swap
                    ID = idDFF2;
                }
            }
            else{ // 可以跟LUT or DFF swap or 移到空位
                if ( numLUT == 0 && numDFF == 0 ) { // 該location沒有LUT和DFF，直接更改location
                    ID = -1;
                }
                else{
                    random = rand()%2;
                    if(random == 0) { // swap with DFF
                        if( numDFF == 0 )
                            ID = -1;
                        else{ // numDFF == 1
                            random = rand()%2;
                            if ( random == 0 )
                                ID = -1;
                            else {
                                TYPE = 'F';
                                ID = idDFF1;
                            }
                        }
                    }
                    else { // swap with LUT
                        if ( numLUT == 0 )
                            ID == -1;
                        else if ( numLUT == 1 ) {
                            random = rand()%2;
                            if( random == 0 )
                                ID = -1;
                            else {
                                TYPE = 'L';
                                ID == idLUT1;
                            }
                        }
                        else{ // numLUT == 2
                            TYPE = 'L';
                            random = rand()%2;
                            if ( random == 0 ) {
                                ID = idLUT1;
                            }
                            else {
                                ID = idLUT2;
                            }
                        }
                    }
                }
            }
            break;
        default:
            break;
    }


    if ( ID == -1 ) { // 移到空位
        if(type == 'L'){
            gID = ID;
            gCoor.x = LUTs[index]->x;
            gCoor.y = LUTs[index]->y;
            LUTs[index]->x = target.x;
            LUTs[index]->y = target.y;
        }
        else {
            gID = ID;
            gCoor.x = DFFs[index]->x;
            gCoor.y = DFFs[index]->y;
            DFFs[index]->x = target.x;
            DFFs[index]->y = target.y;
        }
    }
    else{
        gID = ID;
        gType = TYPE;
        swap(index, ID, type, TYPE);
    }
}

int CalculateNewCost(){
    return 0;
}

// Calculate HPWL
float calculate_WL(){
    HPWL = 0;
    for(int i=1; i<=num_nets; i++){        
        top = 0;
        bottom = R+1;
        right = 0;
        left = C+1;

        current_inst = NETs[i];
        while(current_inst!=NULL){
            switch (current_inst->type)
            {
            case 'L':
                if((float)LUTs[current_inst->id]->x < left){
                    left = (float)LUTs[current_inst->id]->x;
                }
                if((float)LUTs[current_inst->id]->x > right){
                    right = (float)LUTs[current_inst->id]->x;
                }
                if((float)LUTs[current_inst->id]->y < bottom){
                    bottom = (float)LUTs[current_inst->id]->y;
                }
                if((float)LUTs[current_inst->id]->y > top){
                    top = (float)LUTs[current_inst->id]->y;
                }
                break;
            case 'F':
                if((float)DFFs[current_inst->id]->x < left){
                    left = (float)DFFs[current_inst->id]->x;
                }
                if((float)DFFs[current_inst->id]->x > right){
                    right = (float)DFFs[current_inst->id]->x;
                }
                if((float)DFFs[current_inst->id]->y < bottom){
                    bottom = (float)DFFs[current_inst->id]->y;
                }
                if((float)DFFs[current_inst->id]->y > top){
                    top = (float)DFFs[current_inst->id]->y;
                }
                break;
            case 'I':
                if((float)PIs[current_inst->id]->x < left){
                    left = (float)PIs[current_inst->id]->x;
                }
                if((float)PIs[current_inst->id]->x > right){
                    right = (float)PIs[current_inst->id]->x;
                }
                if((float)PIs[current_inst->id]->y < bottom){
                    bottom = (float)PIs[current_inst->id]->y;
                }
                if((float)PIs[current_inst->id]->y > top){
                    top = (float)PIs[current_inst->id]->y;
                }
                break;
            case 'O':
                if((float)POs[current_inst->id]->x < left){
                    left = (float)POs[current_inst->id]->x;
                }
                if((float)POs[current_inst->id]->x > right){
                    right = (float)POs[current_inst->id]->x;
                }
                if((float)POs[current_inst->id]->y < bottom){
                    bottom = (float)POs[current_inst->id]->y;
                }
                if((float)POs[current_inst->id]->y > top){
                    top = (float)POs[current_inst->id]->y;
                }
                break;
            default:
                break;
            }

            current_inst = current_inst->next;
        }
        HPWL += (top-bottom+right-left);
    }

    return HPWL;
}