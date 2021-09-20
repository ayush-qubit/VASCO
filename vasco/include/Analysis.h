#ifndef ANALYSIS_H
#define ANALYSIS_H
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Instructions.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/IR/CFG.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/IRBuilder.h"
#include <string>
#include <stack>
#include <map>
#include <set>
#include <fstream>
#include <iostream>
// #include <bprinter/table_printer.h>

#define RST "\033[0;m"
#define BLACKB "\033[1;90m"
#define REDB "\033[1;91m"
#define GREENB "\033[1;92m"
#define YELLOWB "\033[1;93m"
#define BLUEB "\033[1;94m"
#define MAGENTAB "\033[1;95m"
#define CYANB "\033[1;96m"
#define WHITEB "\033[1;97m"

// #define BLACK "\u001b[;30m"
// #define RED: "\u001b[;31m"
// #define GREEN: "\u001b[;32m"
// #define YELLOW: "\u001b[;33m"
// #define BLUE: "\u001b[;34m"
// #define MAGENTA: "\u001b[;35m"
// #define CYAN: "\u001b[;36m"
// #define WHITE: "\u001b[;37m"
using namespace llvm;
using namespace std;
enum NoAnalysisType {NoAnalyisInThisDirection};

template<class F,class B>
class Analysis
{
    private:
	// F forward_component;
 //    B backward_component;
    Module *current_module;
    int context_label_counter;
    int current_analysis_direction;//0:initial pass 1:forward, 2:backward
    int processing_context_label;
    std::map<pair<int,Instruction*>,pair<F,B>>IN,OUT;
    // std::map<pair<Context<F,B>>>;
    //mapping from context label to context object
    map<int,pair<Function*,pair<pair<F,B>,pair<F,B>>>>context_label_to_context_object_map;
    
    //mapping from context object to context label
    //mapping from function to  pair<inflow,outflow>
    //inflow and outflow are themselves pairs of forward and backward component values.
    //The forward and backward components are themselves pairs of G,L dataflow values.
    // Context<F,B> context;
    map<pair<Function*,pair<pair<F,B>,pair<F,B>>>,int>context_object_to_context_label_map;
	
    protected:
	// std::map<Instruction*,std::pair<F,B>>IN,OUT;
    // std::map<InstructionWrapper*,std::pair<F,B>>IN,OUT;
    
    //debugging purpose
    std::map<BasicBlock*,std::pair<F,B>>BB_IN,BB_OUT;
    
    //List of contexts
    set<int> ProcedureContext;


    // mapping from (context label,basic block) to (forward and backward) data flow values
    map<pair<int,BasicBlock*>,std::pair<F,B>>CS_BB_IN,CS_BB_OUT;

    // worklist of (context label,basic block) for both directions of analysis
    stack<pair<int,BasicBlock*>>forward_worklist,backward_worklist,temp;
    
    // mapping to check which entries are already part of the worklist
    map<pair<int,BasicBlock*>,bool>forward_worklist_contains_this_entry;
    map<pair<int,BasicBlock*>,bool>backward_worklist_contains_this_entry;
    
    // mapping from (context label,basic block) to target context label
    // map<pair<pair<int,BasicBlock*>,int>,int>transitions;
    
    // mapping from (context label,call site) to target context label
    map<pair<int,Instruction*>,int>context_transition_graph;//graph
	
    public:
    Analysis();
	int getContextLabelCounter();
    void setContextLabelCounter(int);
    int getCurrentAnalysisDirection();
    void setCurrentAnalysisDirection(int);
    int getProcessingContextLabel();
    void setProcessingContextLabel(int);
    
    void doAnalysis(Module &M);
    void INIT_CONTEXT(pair<Function*,pair<pair<F,B>,pair<F,B>>> context_object);
    void doAnalysisForward();
    void doAnalysisBackward();
    F NormalFlowFunctionForward(pair<int,BasicBlock*>);
    B NormalFlowFunctionBackward(pair<int,BasicBlock*>);
    int check_if_context_already_exists(pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object);
    void drawSuperGraph(Module &M);
    void printWorklistMaps();
    // bool isAnIgnorableDebugInstruction(std::string mainStr, std::string toMatch);
    bool isAnIgnorableDebugInstruction(Instruction *);
    void performSplittingBB(Function &f);
    //void performFormalToActualMapping(Function &f);

    void setCurrentModule(Module *);
    Module * getCurrentModule();

    F getForwardComponentAtInOfThisInstruction(Instruction &I);
    B getBackwardComponentAtInOfThisInstruction(Instruction &I);
    F getForwardComponentAtOutOfThisInstruction(Instruction &I);
    B getBackwardComponentAtOutOfThisInstruction(Instruction &I);

    void setForwardComponentAtInOfThisInstruction(Instruction* I,F in_value);
    void setBackwardComponentAtInOfThisInstruction(Instruction* I,B in_value);
    void setForwardComponentAtOutOfThisInstruction(Instruction* I,F out_value);
    void setBackwardComponentAtOutOfThisInstruction(Instruction* I,B out_value);

    F getForwardInflowForThisContext(int);
    B getBackwardInflowForThisContext(int);
    F getForwardOutflowForThisContext(int);
    B getBackwardOutflowForThisContext(int);

    void setForwardInflowForThisContext(int,F);
    void setBackwardInflowForThisContext(int,B);
    void setForwardOutflowForThisContext(int,F);
    void setBackwardOutflowForThisContext(int,B);

    void printModule(Module &M);

    Function* getFunctionAssociatedWithThisContext(int);
    void setFunctionAssociatedWithThisContext(int,Function*);

    void printContext();

    virtual pair<F,B> CallInflowFunction(int,Function*,BasicBlock*,F,B);
    virtual pair<F,B> CallOutflowFunction(int,Function*,BasicBlock*,F,B,F,B);

    //pair<F,B> CallInflowFunction(int,Function*,BasicBlock*,F,B);
    //pair<F,B> CallOutflowFunction(int,Function*,BasicBlock*,F,B);


    virtual void printResults(){}
	
	virtual F computeOutFromIn(Instruction &I);
    virtual F getBoundaryInformationForward();//{}
    virtual F getInitialisationValueForward();//{}
    virtual F performMeetForward(F d1,F d2);//{}
    virtual bool EqualDataFlowValuesForward(F d1,F d2);//{}
    // virtual F getPurelyLocalComponentForward(F dfv);
    virtual F getPurelyGlobalComponentForward(F dfv);
    // virtual F getMixedComponentForward(F dfv);
    // virtual F getCombinedValuesAtCallForward(F dfv1,F dfv2);

    virtual void printDataFlowValuesForward(F dfv){}

    virtual B computeInFromOut(Instruction &I);
    virtual B getBoundaryInformationBackward();//{}
    virtual B getInitialisationValueBackward();//{}
    virtual B performMeetBackward(B d1,B d2);//{}
    virtual bool EqualDataFlowValuesBackward(B d1,B d2);//{}
    // virtual B getPurelyLocalComponentBackward(B dfv);
    virtual B getPurelyGlobalComponentBackward(B dfv);
    // virtual B getMixedComponentBackward(B dfv);
    // virtual B getCombinedValuesAtCallBackward(B dfv1,B dfv2);
    
    virtual void printDataFlowValuesBackward(B dfv){}
    
    // void printInOutMapsBackward(Function &function);
    // void printInOutMapsForward(Function &function);
    virtual void printInOutMaps(){}
    virtual void printForwardWorklist(){}
    virtual void printBackwardWorklist(){}
    


};

//========================================================================================
// template<class F,class B>
// bool Analysis<F,B>::isAnIgnorableDebugInstruction(std::string mainStr, std::string toMatch)
// {
//     // std::string::find returns 0 if toMatch is found at starting
//         if(mainStr.find(toMatch) == 0)
//             return true;
//         else
//             return false;
// }
template<class F,class B>
Analysis<F,B>::Analysis(){
    current_module=nullptr;
    context_label_counter=-1;
}

template<class F,class B>
int Analysis<F,B>::getContextLabelCounter()
{
    return context_label_counter;
}

template<class F,class B>
void Analysis<F,B>::setContextLabelCounter(int new_context_label_counter)
{
    context_label_counter=new_context_label_counter;
}

template<class F,class B>
int Analysis<F,B>::getCurrentAnalysisDirection()
{
    return current_analysis_direction;
}

template<class F,class B>
void Analysis<F,B>::setCurrentAnalysisDirection(int direction)
{
    current_analysis_direction=direction;
}

template<class F,class B>
int Analysis<F,B>::getProcessingContextLabel()
{
    return processing_context_label;
}

template<class F,class B>
void Analysis<F,B>::setProcessingContextLabel(int label)
{
    processing_context_label=label;
}

template<class F,class B>
bool Analysis<F,B>::isAnIgnorableDebugInstruction(Instruction *inst)
{
    if(isa<DbgDeclareInst>(inst) || isa<DbgValueInst>(inst)){
        return true;
    }
    return false;
}

template <class F,class B>
Module* Analysis<F,B>::getCurrentModule()
{
    return current_module;
}

template <class F,class B>
void Analysis<F,B>::setCurrentModule(Module *m)
{
    current_module=m;
}

// template <class F,class B>
// void Analysis<F,B>::printDataFlowValuesForward(F dfv)
// {
    
// }

// template <class F,class B>
// void Analysis<F,B>::printDataFlowValuesBackward(F dfv)
// {
    
// }

template <class F,class B>
B Analysis<F,B>::computeInFromOut(Instruction &I)
{
    errs()<<"\nThis function computeInFromOut() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::computeOutFromIn(Instruction &I)
{
    errs()<<"\nThis function computeOutFromIn() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getBoundaryInformationForward(){
    errs()<<"\nThis function getBoundaryInformationForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
B Analysis<F,B>::getBoundaryInformationBackward(){
    errs()<<"\nThis function getBoundaryInformationBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getInitialisationValueForward(){
    errs()<<"\nThis function getInitialisationValueForward() has not been implemented. EXITING !!\n";
    exit(-1);   
}

template <class F,class B>
B Analysis<F,B>::getInitialisationValueBackward(){
    errs()<<"\nThis function getInitialisationValueBackward() has not been implemented. EXITING !!\n";
    exit(-1);   
}

template <class F,class B>
F Analysis<F,B>::performMeetForward(F d1,F d2){
    errs()<<"\nThis function performMeetForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
B Analysis<F,B>::performMeetBackward(B d1,B d2){
    errs()<<"\nThis function performMeetBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
bool Analysis<F,B>::EqualDataFlowValuesForward(F d1,F d2){
    errs()<<"\nThis function EqualDataFlowValuesForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
bool Analysis<F,B>::EqualDataFlowValuesBackward(B d1,B d2){
    errs()<<"\nThis function EqualDataFlowValuesBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
B Analysis<F,B>::getPurelyGlobalComponentBackward(B dfv)
{
    errs()<<"\nThis function getPurelyGlobalComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
F Analysis<F,B>::getPurelyGlobalComponentForward(F dfv)
{
    errs()<<"\nThis function getPurelyGlobalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}
/*
template <class F,class B>
B Analysis<F,B>::getPurelyLocalComponentBackward(B dfv)
{
    errs()<<"\nThis function getPurelyLocalComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
F Analysis<F,B>::getPurelyLocalComponentForward(F dfv)
{
    errs()<<"\nThis function getPurelyLocalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}



template <class F,class B>
B Analysis<F,B>::getMixedComponentBackward(B dfv)
{
    errs()<<"\nThis function getMixedComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getMixedComponentForward(F dfv)
{
    errs()<<"\nThis function getMixedComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
B Analysis<F,B>::getCombinedValuesAtCallBackward(B dfv1,B dfv2)
{
    errs()<<"\nThis function getCombinedValuesAtCallBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getCombinedValuesAtCallForward(F dfv1,F dfv2)
{
    errs()<<"\nThis function getCombinedValuesAtCallForward() has not been implemented. EXITING !!\n";
    exit(-1);
}
*/
//========================================================================================

template<class F,class B>
void Analysis<F,B>::printModule(Module &M){
    outs() << "--------------------------------------------" << "\n";
    for(Function& Func : M){
        outs() << "FUNCTION NAME: ";
        outs() << Func.getName() << "\n";
        outs() << "----------------------------" << "\n";
        for(BasicBlock& BB : Func){
            outs() << "----------------------" << "\n";
            for(Instruction& inst : BB){
                outs() << inst << "\n";
            }
            outs() << "----------------------" << "\n";
        }
        outs() << "-----------------------------" << "\n";
    }
    outs() << "---------------------------------------------" << "\n";
}


template<class F,class B>
pair<F,B> Analysis<F,B>::CallInflowFunction(int context_label,Function* target_function,BasicBlock* bb,F a1,B d1)
{
    errs()<<"\nThis function CallInflowFunction() has not been implemented. EXITING !!\n";
    exit(-1);
    // F a2;
    // B d2;
    // if(0==getCurrentAnalysisDirection()||2==getCurrentAnalysisDirection())
    // {
    //     a2=a1;
    //     d2=getPurelyLocalComponentBackward(d1);
    // }
    // else
    // {
    //     a2=getPurelyGlobalComponentForward(a1);
    //     d2=d1;
    // }
    // // F a2=getPurelyGlobalComponentForward(a1);
    // // B d2=d1;
    // return make_pair(a2,d2);
}

template<class F,class B>
pair<F,B> Analysis<F,B>::CallOutflowFunction(int context_label,Function* target_function,BasicBlock* bb,F a3,B d3,F a1,B d1)
{
    errs()<<"\nThis function CallOutflowFunction() has not been implemented. EXITING !!\n";
    exit(-1);
    // F a4;
    // B d4;
    // if(0==getCurrentAnalysisDirection()||2==getCurrentAnalysisDirection())
    // {
    //     //backward
    //     a4=a3;
    //     d4=getPurelyGlobalComponentBackward(d3);
    // }
    // else
    // {
    //     //forward 
    //     a4=getPurelyGlobalComponentForward(a4);
    //     d4=d3;
    // }
    // // F a4=a3;
    // // B d4=getPurelyGlobalComponentBackward(d3);
    // return make_pair(a4,d4);
}

//=====================setter and getters for IN-OUT Maps==================================
template<class F,class B>
F Analysis<F,B>::getForwardComponentAtInOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return IN[make_pair(label,&I)].first;
}

template<class F,class B>
F Analysis<F,B>::getForwardComponentAtOutOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return OUT[make_pair(label,&I)].first;
}

template<class F,class B>
B Analysis<F,B>::getBackwardComponentAtInOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return IN[make_pair(label,&I)].second;
}

template<class F,class B>
B Analysis<F,B>::getBackwardComponentAtOutOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return OUT[make_pair(label,&I)].second;
}

template<class F,class B>
void Analysis<F,B>::setForwardComponentAtInOfThisInstruction(Instruction *I,F in_value)
{
    int label=getProcessingContextLabel();
    IN[make_pair(label,I)].first=in_value;
}

template<class F,class B>
void Analysis<F,B>::setForwardComponentAtOutOfThisInstruction(Instruction *I,F out_value)
{
    int label=getProcessingContextLabel();
    OUT[make_pair(label,I)].first=out_value;
}

template<class F,class B>
void Analysis<F,B>::setBackwardComponentAtInOfThisInstruction(Instruction *I,B in_value)
{
    int label=getProcessingContextLabel();
    IN[make_pair(label,I)].second=in_value;
}

template<class F,class B>
void Analysis<F,B>::setBackwardComponentAtOutOfThisInstruction(Instruction *I,B out_value)
{
    int label=getProcessingContextLabel();
    OUT[make_pair(label,I)].second=out_value;
}


//=====================setter and getters for context objects==================================
template<class F,class B>
F Analysis<F,B>::getForwardInflowForThisContext(int context_label)
{
    return context_label_to_context_object_map[context_label].second.first.first;
}

template<class F,class B>
B Analysis<F,B>::getBackwardInflowForThisContext(int context_label)
{
    return context_label_to_context_object_map[context_label].second.first.second;
}

template<class F,class B>
F Analysis<F,B>::getForwardOutflowForThisContext(int context_label)
{
    return context_label_to_context_object_map[context_label].second.second.first;
}

template<class F,class B>
B Analysis<F,B>::getBackwardOutflowForThisContext(int context_label)
{
    return context_label_to_context_object_map[context_label].second.second.second;
}


template<class F,class B>
void Analysis<F,B>::setForwardInflowForThisContext(int context_label,F forward_inflow)
{
    context_label_to_context_object_map[context_label].second.first.first=forward_inflow;
}

template<class F,class B>
void Analysis<F,B>::setBackwardInflowForThisContext(int context_label,B backward_inflow)
{
    context_label_to_context_object_map[context_label].second.first.second=backward_inflow;
}

template<class F,class B>
void Analysis<F,B>::setForwardOutflowForThisContext(int context_label,F forward_outflow)
{
    context_label_to_context_object_map[context_label].second.second.first=forward_outflow;
}

template<class F,class B>
void Analysis<F,B>::setBackwardOutflowForThisContext(int context_label,B backward_outflow)
{
    context_label_to_context_object_map[context_label].second.second.second=backward_outflow;
}

template<class F,class B>
Function* Analysis<F,B>::getFunctionAssociatedWithThisContext(int context_label)
{
    return context_label_to_context_object_map[context_label].first;
}

template<class F,class B>
void Analysis<F,B>::setFunctionAssociatedWithThisContext(int context_label,Function *f)
{
    context_label_to_context_object_map[context_label].first=f;
}

//================================================================================================
template <class F,class B>
void Analysis<F,B>::doAnalysis(Module &M)
{
    setCurrentModule(&M);
    // ======================================================================================

    //====================================SPLITTING========================================
    for(Function &function: M)
	{
		errs()<<BLUEB"\n----------------------------------------------------------------\n";
		errs()<<REDB<<function.getName()<<RST;
        // if(!isAnIgnorableDebugInstruction(function.getName().str(),"llvm.dbg"))
        // {
        //     if(function.size()>0)
        //         performSplittingBB(function);

        // }
        // else
        // {
        //     errs()<<"\nDebug Instr! Skipping!!";
        // }
        if(function.size()>0){
            performSplittingBB(function);
        }
		errs()<<BLUEB"\n----------------------------------------------------------------\n";
	}
    
	errs()<<RST;
    // return;
    //======================================================================================
    printModule(M);

    int i=0;
    for(Function &function: M)
	{
		errs() <<"\nMy code: Function Fetched: "<<++i<<". "<< function.getName();
		if(function.getName()=="main")
		{
			errs()<<"\n"<<REDB<<"Main Found!"<<RST;
            F forward_inflow_bi;//=getBoundaryInformationForward();
            B backward_inflow_bi;//=getBoundaryInformationBackward();
            F forward_outflow_bi;
            B backward_outflow_bi;
            Function *fptr=&function;
            if(std::is_same<F, NoAnalysisType>::value)
            {
                // forward_bi=NoAnalyisInThisDirection;   
            }
            else
            {
                forward_inflow_bi=getBoundaryInformationForward();
            }
            if(std::is_same<B, NoAnalysisType>::value)
            {
                // forward_bi=NoAnalyisInThisDirection;   
            }
            else
            {
                backward_inflow_bi=getBoundaryInformationBackward();
            }
            setCurrentAnalysisDirection(0);
            INIT_CONTEXT(make_pair(fptr,make_pair(make_pair(forward_inflow_bi,backward_inflow_bi),make_pair(forward_outflow_bi,backward_outflow_bi))));
            errs()<<REDB"\nSize of Worklist:"<<forward_worklist.size()<<" and "<<backward_worklist.size()<<RST;
		}
    }
    errs()<<REDB<<"\nFWS:"<<forward_worklist.size()<<" BWS:"<<backward_worklist.size()<<RST;
    errs()<<"\n------------------------BEGIN ANALYSIS----------------------------------";
    if(std::is_same<F, NoAnalysisType>::value)
    {
        //backward analysis
        setCurrentAnalysisDirection(2);
        int backward_iteration_count=0;
        int iteration = 1;
        while (backward_worklist.size()>0)
        {
            errs() << REDB << "\n--------------------------------------Start Iteration-" << iteration << "--------------------------------------\n";
            doAnalysisBackward();
            backward_iteration_count++;
            printContext();
            errs() << REDB << "\n--------------------------------------End Iteration-" << iteration++ << "--------------------------------------\n";
        }
        errs()<<REDB<<"\nbackward_iteration_count:"<<backward_iteration_count;
    }
    else if(std::is_same<B, NoAnalysisType>::value)
    {
        //forward analysis
        setCurrentAnalysisDirection(1);
        int forward_iteration_count=0;
        int iteration = 1;
        while (forward_worklist.size()>0)
        {
            // errs() << REDB << "\n--------------------------------------Start Iteration-" << iteration << "--------------------------------------\n";
            doAnalysisForward();
            forward_iteration_count++;
            // printContext();
            // errs() << REDB << "\n--------------------------------------End Iteration-" << iteration++ << "--------------------------------------\n";
        }
        printContext();
        errs()<<MAGENTAB"\nforward_iteration_count:"<<forward_iteration_count<<" FW size:"<<forward_worklist.size()<<RST;
    }
    else
    {
        int fi=1,bi=1;
        int iteration = 1;
        while (forward_worklist.size()>0||backward_worklist.size()>0)
        { 
            errs() << REDB << "\n--------------------------------------Start Iteration-" << iteration << "--------------------------------------\n"; 
            errs()<<MAGENTAB<<"\nBackward Iteration:"<<bi++<<RST;
            // current_analysis_direction=2;
            setCurrentAnalysisDirection(2);
            while(backward_worklist.size()>0)
            {
                doAnalysisBackward();
            }
            // for(auto val : context_label_to_context_object_map){
            //     outs() << "\n";
            //     outs() << "LABEL: " << val.first << " ";
            //     outs() << "FUNCTION NAME: " << val.second.first->getName() << " ";
            //     // tp << val.first << val.second->getName();
            //     outs() << "INFLOW: " << "(";
            //     for(auto v : val.second.second.first.first){
            //         outs() << v;
            //     }
            //     outs() << ",";
            //     for(auto v : val.second.second.first.second){
            //         outs() << v;
            //     }
            //     outs() << ")" << " ";
            //     outs() << "OUTFLOW: " << "(";
            //     for(auto v : val.second.second.second.first){
            //         outs() << v;
            //     }
            //     outs() << ",";
            //     for(auto v : val.second.second.second.second){
            //         outs() << v;
            //     }
            //     outs() << ")";
            //     outs() << "\n";
            // }
            // for(auto val : context_transition_graph){
            //     outs() << val.first.first << "->" << val.second << "\n";
            // }
            errs()<<MAGENTAB<<"\nForward Iteration:"<<fi++<<RST;
            // current_analysis_direction=1;
            setCurrentAnalysisDirection(1);
            while(forward_worklist.size()>0)
            {
                doAnalysisForward();
            }
            // for(auto val : context_label_to_context_object_map){
            //     outs() << "\n";
            //     outs() << "LABEL: " << val.first << " ";
            //     outs() << "FUNCTION NAME: " << val.second.first->getName() << " ";
            //     outs() << "INFLOW: " << "(";
            //     for(auto v : val.second.second.first.first){
            //         outs() << v;
            //     }
            //     outs() << ",";
            //     for(auto v : val.second.second.first.second){
            //         outs() << v;
            //     }
            //     outs() << ")" << " ";
            //     outs() << "OUTFLOW: " << "(";
            //     for(auto v : val.second.second.second.first){
            //         outs() << v;
            //     }
            //     outs() << ",";
            //     for(auto v : val.second.second.second.second){
            //         outs() << v;
            //     }
            //     outs() << ")";
            //     outs() << "\n";
            // }
            // for(auto val : context_transition_graph){
            //     outs() << val.first.first << "->" << val.second << "\n";
            // }
            // outs() << "FORWARD WORKLIST" << "\n";
            // printForwardWorklist();
            // outs() <<  "BACKWARD_WORKLIST" << "\n";
            // printBackwardWorklist();
            printContext();
            errs() << REDB << "\n--------------------------------------End Iteration-" << iteration << "--------------------------------------\n";
            iteration++;
        }
    }
    errs()<<REDB<<"\nTotal Contexts created:"<<ProcedureContext.size()<<" Context Counter"<<context_label_counter<<RST;
    errs()<<REDB<<"\nCS_BB_IN size:"<<CS_BB_IN.size()<<" and CS_BB_OUT size:"<<CS_BB_OUT.size()<<RST;
    printResults();
    // printInOutMaps();
}

template <class F,class B>
void Analysis<F,B>::INIT_CONTEXT(pair<Function*,pair<pair<F,B>,pair<F,B>>> context_object)//,F forward_component,B backward_component)
{
    Function &function=*(context_object.first);
    errs()<<REDB<<"\nInit Context called! for function: "<<function.getName()<<RST;
    outs() << "\n";
    context_label_counter++;
    int current_context_label=context_label_counter;
    setProcessingContextLabel(current_context_label);

    if(std::is_same<F, NoAnalysisType>::value)
    {
        //backward analysis
        
        context_label_to_context_object_map[current_context_label].first=&function;
        // errs()<<MAGENTAB<<"\nSetting Inflow Backward:"<<context_object.second.first.second[0]<<" for label:"<<current_context_label;
        // context_label_to_context_object_map[current_context_label].second.first.second=context_object.second.first.second;//setting inflow backward
        setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
        //context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
        context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
        ProcedureContext.insert(current_context_label);

        for(BasicBlock *BB:inverse_post_order(&function.back()))
        {
            BasicBlock &b=*BB;
            errs()<<"\nBasicBlock: ";
            b.printAsOperand(errs(),false);
            forward_worklist.push(make_pair(current_context_label,&b));
            backward_worklist.push(make_pair(current_context_label,&b));
            forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
            backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
            // CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
            CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
            // CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
            CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
            
            //initialise IN-OUT maps for every instruction
            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
            {
                errs()<<"\n"<<*inst;
                setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                // IN[&(*inst)].second=getInitialisationValueBackward();
                // OUT[&(*inst)].second=getInitialisationValueBackward();
            }
        }
        if(current_context_label==0)//main function with first invocation
        {
            setBackwardInflowForThisContext(current_context_label,getBoundaryInformationBackward());//setting inflow backward
            // context_label_to_context_object_map[current_context_label].second.first.second=getBoundaryInformationBackward();//setting inflow backward
        }
        else
        {
            setBackwardInflowForThisContext(current_context_label,context_object.second.first.second);//setting inflow backward
            // context_label_to_context_object_map[current_context_label].second.first.second=context_object.second.first.second;//setting inflow backward
        }
        // errs()<<MAGENTAB<<"\nSetting CS BB OUT Backward:"<<context_object.second.first.second[0]<<" for label:"<<current_context_label;
        CS_BB_OUT[make_pair(current_context_label,&function.back())].second=getBackwardInflowForThisContext(current_context_label);
        // CS_BB_OUT[make_pair(current_context_label,&function.back())].second=context_label_to_context_object_map[current_context_label].second.first.second;

    }
    else if(std::is_same<B, NoAnalysisType>::value)
    {
        //forward analysis
        context_label_to_context_object_map[current_context_label].first=&function;
        // context_label_to_context_object_map[current_context_label].second.first.first=context_object.second.first.first;//setting inflow forward
        
        setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
        // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
        context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
        ProcedureContext.insert(current_context_label);
        
        errs()<<CYANB"\nNumber of Basic Blocks:"<<function.size()<<RST;
        for(BasicBlock *BB:post_order(&function.getEntryBlock()))
        {
            BasicBlock &b=*BB;
            errs()<<"\nBasicBlock: ";
            b.printAsOperand(errs(),false);
            forward_worklist.push(make_pair(current_context_label,&b));
            backward_worklist.push(make_pair(current_context_label,&b));
            forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
            backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
            CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
            // CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
            CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
            // CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
            
            //initialise IN-OUT maps for every instruction
            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
            {
                errs()<<"\n"<<*inst;
                setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                // IN[&(*inst)].first=getInitialisationValueForward();
                // OUT[&(*inst)].first=getInitialisationValueForward();
            }
        }
        if(current_context_label==0)//main function with first invocation
        {
            setForwardInflowForThisContext(current_context_label,getBoundaryInformationForward());//setting inflow forward
            // context_label_to_context_object_map[current_context_label].second.first.first=getBoundaryInformationForward();//setting inflow forward
        }
        else
        {
            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);//setting inflow forward
            // context_label_to_context_object_map[current_context_label].second.first.first=context_object.second.first.first;//setting inflow forward
        }
        CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
        // CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=context_label_to_context_object_map[current_context_label].second.first.first;
    }
    else
    {
        //bidirectional analysis
        context_label_to_context_object_map[current_context_label].first=&function;

        if(getCurrentAnalysisDirection()==10)
        {
            errs()<<CYANB<<"\nforward init-context"<<RST;
            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
            
            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
            ProcedureContext.insert(current_context_label);
            for(BasicBlock *BB:post_order(&function.getEntryBlock()))
            {
                BasicBlock &b=*BB;
                // errs()<<"\nBasicBlock: ";
                // b.printAsOperand(errs(),false);
                forward_worklist.push(make_pair(current_context_label,&b));
                // backward_worklist.push(make_pair(current_context_label,&b));
                forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                // backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                
                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
                {
                    // errs()<<"\n"<<*inst;
                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
        }
        else if(getCurrentAnalysisDirection()==20)
        {
            errs()<<CYANB<<"\nbackward init-context"<<RST;
            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward    
            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
            
            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
            ProcedureContext.insert(current_context_label);
            for(BasicBlock *BB:inverse_post_order(&function.back()))
            {
                BasicBlock &b=*BB;
                // errs()<<"\nBasicBlock: ";
                // b.printAsOperand(errs(),false);
                // forward_worklist.push(make_pair(current_context_label,&b));
                backward_worklist.push(make_pair(current_context_label,&b));
                // forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                
                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
                {
                    // errs()<<"\n"<<*inst;
                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
            
        }
        else
        {
            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
            
            
            
            
            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
            ProcedureContext.insert(current_context_label);
            for(BasicBlock *BB:inverse_post_order(&function.back()))
            {
                //populate backward worklist
                BasicBlock &b=*BB;
                // errs()<<"\nBasicBlock: ";
                // b.printAsOperand(errs(),false);
                // forward_worklist.push(make_pair(current_context_label,&b));
                backward_worklist.push(make_pair(current_context_label,&b));
                // forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                // CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                // CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                
                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
                {
                    // errs()<<"\n"<<*inst;
                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    // setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
            for(BasicBlock *BB:post_order(&function.getEntryBlock()))
            {
                //populate forward worklist
                BasicBlock &b=*BB;
                // errs()<<"\nBasicBlock: ";
                // b.printAsOperand(errs(),false);
                forward_worklist.push(make_pair(current_context_label,&b));
                // backward_worklist.push(make_pair(current_context_label,&b));
                forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                // backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                // CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                // CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                
                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
                {
                    // errs()<<"\n"<<*inst;
                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
        }
        //irresepective of the current direction of analysis, the INFLOW values need to be set.
        if(current_context_label==0)//main function with first invocation
        {
            setForwardInflowForThisContext(current_context_label,getBoundaryInformationForward());//setting inflow forward
            setBackwardInflowForThisContext(current_context_label,getBoundaryInformationBackward());//setting inflow backward
            // context_label_to_context_object_map[current_context_label].second.first.second=getBoundaryInformationBackward();//setting inflow backward
            // context_label_to_context_object_map[current_context_label].second.first.first=getBoundaryInformationForward();//setting inflow forward
        }
        else
        {
            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);//setting inflow forward
            setBackwardInflowForThisContext(current_context_label,context_object.second.first.second);//setting inflow backward
            // context_label_to_context_object_map[current_context_label].second.first.first=context_object.second.first.first;//setting inflow forward
            // context_label_to_context_object_map[current_context_label].second.first.second=context_object.second.first.second;//setting inflow backward
        }
        CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
        CS_BB_OUT[make_pair(current_context_label,&function.back())].second=getBackwardInflowForThisContext(current_context_label);
        // CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=context_label_to_context_object_map[current_context_label].second.first.first;
        // CS_BB_OUT[make_pair(current_context_label,&function.back())].second=context_label_to_context_object_map[current_context_label].second.first.second;
    
    }
    errs()<<REDB<<"\nInit Context ENDED! for function: "<<function.getName()<<RST;
}

template <class F,class B>
void Analysis<F,B>::doAnalysisForward()
{
    while(!forward_worklist.empty())//step 2
    {
        //step 3 and 4
        pair<int,BasicBlock*> current_pair=forward_worklist.top();
        int current_context_label;
        BasicBlock *bb;
        current_context_label=forward_worklist.top().first;
        setProcessingContextLabel(current_context_label);
        bb=forward_worklist.top().second;
        forward_worklist.pop();
        forward_worklist_contains_this_entry[make_pair(current_context_label,bb)]=false;

        BasicBlock &b=*bb;
        Function *f=context_label_to_context_object_map[current_context_label].first;
        Function &function=*f;
        errs()<<BLUEB"\nContext Label: "<<current_context_label<<" BasicBlock: ";
        b.printAsOperand(errs(),false);
        errs()<<" Function Name: "<<f->getName()<<" WS:"<<forward_worklist.size()<<RST;
        
        //step 5
        if(bb!=(&function.getEntryBlock()))
        {
            errs()<<"\nNon-Entry Block\n";
            //step 6
            CS_BB_IN[current_pair].first=getInitialisationValueForward();
            
            //step 7 and 8
            for(auto pred_bb:predecessors(bb))
            {
                errs()<<"\nPred BB: ";
                pred_bb->printAsOperand(errs(),false);
                errs() << "\t"; //Label of Basic block
                CS_BB_IN[current_pair].first=performMeetForward(CS_BB_IN[current_pair].first,CS_BB_OUT[make_pair(current_pair.first,pred_bb)].first);                
            }
        }
        else
        {
            errs()<<"\nEntry Block\n";
            //In value of this node is same as INFLOW value
            
            CS_BB_IN[current_pair].first=getForwardInflowForThisContext(current_context_label);
            // CS_BB_IN[current_pair].first=context_label_to_context_object_map[current_context_label].second.first.first;
            
        }
        
        //step 9
        F a1=CS_BB_IN[current_pair].first;
        B d1=CS_BB_OUT[current_pair].second;
        // printDataFlowValuesForward(a1);

        F previous_value_at_out_of_this_node=CS_BB_OUT[current_pair].first;

        //step 10
        bool contains_a_method_call=false;
        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        {
            Instruction &I=*inst;
            if ( CallInst *ci = dyn_cast<CallInst>(&I))
            {
                Function* target_function= ci->getCalledFunction();
                // errs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
                // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
                // {
                //     continue;//this is an inbuilt function so doesnt need to be processed.
                // }
                if((*target_function).size()==0||isAnIgnorableDebugInstruction(&I))
                {
                    continue;//this is an inbuilt function so doesnt need to be processed.
                }
                contains_a_method_call=true;
                errs()<<YELLOWB"\n----------------------------------------------"<<RST;
                errs()<<REDB"\nMethod Call found in basic block: "<<inst->getParent()->getName()<<RST;
                
                break;
            }
        }
        // errs()<<MAGENTAB"\nContains a method call: "<<contains_a_method_call<<RST;
        if(contains_a_method_call)
        {
            F prev=CS_BB_IN[current_pair].first;//a1
            
            
            //step 11
            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
            {
                Instruction &I=*inst;
                if ( CallInst *ci = dyn_cast<CallInst>(&I))
                {
                    Function* target_function= ci->getCalledFunction();
                    // errs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
                    // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
                    // {
                    //     continue;//this is an inbuilt function so doesnt need to be processed.
                    // }
                    if((*target_function).size()==0||isAnIgnorableDebugInstruction(&I))
                    {
                        continue;//this is an inbuilt function so doesnt need to be processed.
                    }
                    
                    errs()<<"\n"<<*inst;
                    
                    /*
                    At the call instruction, the value at IN should be splitted into two components:
                    1) Purely Global and 2) Mixed.
                    The purely global component is given to the start of callee.
                    */
                    
                    //step 12
                    pair<F,B> inflow_pair=CallInflowFunction(current_context_label,target_function,bb,a1,d1);
                    F a2=inflow_pair.first;
                    B d2=inflow_pair.second;

                    // F new_inflow_forward=getPurelyGlobalComponentForward(prev);
                    // B new_inflow_backward;
                    F new_outflow_forward;
                    B new_outflow_backward;
                    
                    //step 13
                    // pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(new_inflow_forward,new_inflow_backward),make_pair(new_outflow_forward,new_outflow_backward)));
                    pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(a2,d2),make_pair(new_outflow_forward,new_outflow_backward)));

                    //===========================================================
                    // IN[&(*inst)].first=prev;//compute IN from previous OUT-value
                    setForwardComponentAtInOfThisInstruction(&(*inst),prev);//compute IN from previous OUT-value
                    //===========================================================
                    // errs()<<WHITEB<<"\nINFLOW:"<<prev[0];
                    
                    int matching_context_label=0;
                    matching_context_label=check_if_context_already_exists(new_context_object);
                    if(matching_context_label>0)//step 15
                    {
                        errs()<<BLUEB"\nExisting context found! It has label: "<<matching_context_label<<RST;
                        outs() << "\n===================================FORWARD===========================================" << "\n";
                        outs() << "INSTRUCTION: " << *inst << "\n";
                        //step 14
                        outs() << "IN: ";
                        pair<int,Instruction*>mypair=make_pair(current_context_label,&(*inst));
                        context_transition_graph[mypair]=matching_context_label;
                        printDataFlowValuesForward(context_label_to_context_object_map[matching_context_label].second.first.first);

                        //step 16 and 17
                        F a3=getForwardOutflowForThisContext(matching_context_label);
                        B d3=getBackwardOutflowForThisContext(matching_context_label);
                        // F a3=context_label_to_context_object_map[matching_context_label].second.second.first;
                        // B d3=context_label_to_context_object_map[matching_context_label].second.second.second;
                        pair<F,B> outflow_pair=CallOutflowFunction(current_context_label,target_function,bb,a3,d3,a1,d1);
                        F value_to_be_meet_with_prev_out=outflow_pair.first;
                        B d4=outflow_pair.second;
                        
                        // new_outflow_forward=getForwardOutflowForThisContext(matching_context_label);
                        
                        // errs()<<WHITEB<<"\nNew Outflow before meet:"<<new_outflow_forward[0]<<RST;
                        

                        //===========================================================
                        //step 18 and 19
                        
                        /*
                        At the call instruction, the value at IN should be splitted into two components.
                        The purely global component is given to the callee while the mixed component is propagated
                        to OUT of this instruction after executing computeOUTfromIN() on it.
                        */
                        
                        // F value_to_be_propagated_to_out_of_instruction=computeOutFromIn(*inst);//a5
                        
                        /*
                        At the OUT of this instruction, the value from END of callee procedure is to be merged
                        with the local(mixed) value propagated from IN. Note that this merging "isn't" 
                        exactly (necessarily) the meet between these two values.
                        */
                        
                        // F merged_value_at_out_of_instruction=getCombinedValuesAtCallForward(new_outflow_forward,value_to_be_propagated_to_out_of_instruction);//combining a4 and a5

                        /*
                        As explained in ip-vasco,pdf, we need to perform meet with the original value of OUT
                        of this instruction to avoid the oscillation problem.
                        */
                        
                        
                        setForwardComponentAtOutOfThisInstruction(&(*inst),performMeetForward(value_to_be_meet_with_prev_out,getForwardComponentAtOutOfThisInstruction(*inst)));
                        
                        // prev=OUT[&(*inst)].first;
                        prev=getForwardComponentAtOutOfThisInstruction((*inst));
                        outs() << "OUT: ";
                        printDataFlowValuesForward(prev);
                        
                        // errs()<<WHITEB<<"\nNew Outflow after meet:"<<prev[0]<<RST;
                        //===========================================================
                    }
                    else//step 20
                    {
                        errs() <<REDB"\n----------------------------------------------\n";
                        errs() << MAGENTAB"CREATING A NEW CONTEXT!!!!!!!" << "\n";
                        //creating a new context
                        INIT_CONTEXT(new_context_object);//step 21

                        //step 14
                        //This step must be done after above step because context label counter gets updated after INIT-Context is invoked.
                        pair<int,Instruction*>mypair=make_pair(current_context_label,&(*inst));
                        context_transition_graph[mypair]=getContextLabelCounter();
                        errs() << "LABEL:- " << context_label_counter << "\n";
                        errs() << "INSTRUCTION:- " << *inst << "\n";
                        errs() <<REDB"\n----------------------------------------------\n";
                        return;
                        //===========================================================
                        // OUT[&(*inst)].first=computeOutFromIn(*inst);
                        // prev=OUT[&(*inst)].first;
                        //===========================================================
                    }
                }
                else
                {
                    outs() << "\n===================================FORWARD===========================================" << "\n";
                    outs() << "INSTRUCTION: " << *inst << "\n";
                    // IN[&(*inst)].first=prev;//compute IN from previous OUT-value
                    outs() << "IN: ";
                    printDataFlowValuesForward(prev);
                    setForwardComponentAtInOfThisInstruction(&(*inst),prev);//compute IN from previous OUT-value
                    // OUT[&(*inst)].first=computeOutFromIn(*inst);
                    F new_prev = computeOutFromIn(*inst);
                    setForwardComponentAtOutOfThisInstruction(&(*inst),new_prev);
                    outs() << "OUT: ";
                    printDataFlowValuesForward(new_prev);
                    // prev=OUT[&(*inst)].first;
                    prev=getForwardComponentAtOutOfThisInstruction(*inst);
                    outs() << "\n===================================FORWARD===========================================" << "\n";
                }
            }
            CS_BB_OUT[current_pair].first=prev;
        }
        else//step 22
        {
            //step 23
            //NormalFlowFunction
            CS_BB_OUT[current_pair].first=NormalFlowFunctionForward(current_pair);
        }
        bool changed=false;
        if(!EqualDataFlowValuesForward(previous_value_at_out_of_this_node,CS_BB_OUT[current_pair].first))
        {
            changed=true;
        }
        if(changed)//step 24
        {
            //not yet converged
            for(auto succ_bb:successors(bb))//step 25
            {
                errs()<<"\nPushing Succ BB: ";
                succ_bb->printAsOperand(errs(),false);
                errs() << "\t"; //Label of Basic block

                //step 26
                if(!forward_worklist_contains_this_entry[make_pair(current_context_label,succ_bb)])
                {
                    forward_worklist.push(make_pair(current_context_label,succ_bb));
                    forward_worklist_contains_this_entry[make_pair(current_context_label,succ_bb)]=true;
                }
                if(!backward_worklist_contains_this_entry[make_pair(current_context_label,succ_bb)])
                {
                    backward_worklist.push(make_pair(current_context_label,succ_bb));
                    backward_worklist_contains_this_entry[make_pair(current_context_label,succ_bb)]=true;
                }
            }

        }
        if(bb==&function.back())//step 27
        {
            //last node/exit node
            
            //step 28
            // errs()<<WHITEB<<"\nOUTFLOW VALUE(before):"<<context_label_to_context_object_map[current_context_label].second.second.first[0];
            

            
            setForwardOutflowForThisContext(current_context_label,getPurelyGlobalComponentForward(CS_BB_OUT[current_pair].first));//setting forward outflow
            context_label_to_context_object_map[current_context_label].second.second.first=getPurelyGlobalComponentForward(CS_BB_OUT[current_pair].first);//setting forward outflow
            
            // errs()<<WHITEB<<"\nOUTFLOW VALUE(after):"<<context_label_to_context_object_map[current_context_label].second.second.first[0];
            
            for(auto context_inst_pairs:context_transition_graph)//step 29
            {
                if(context_inst_pairs.second==current_context_label)//matching the called function
                {
                    //step 30
                    errs()<<CYANB<<"\ncaller basic block label:"<<context_inst_pairs.first.first<<RST;
                    BasicBlock *bb=context_inst_pairs.first.second->getParent();
                    pair<int,BasicBlock*>context_bb_pair=make_pair(context_inst_pairs.first.first,bb);
                    if(!forward_worklist_contains_this_entry[context_bb_pair])
                    {
                        forward_worklist.push(context_bb_pair);
                        forward_worklist_contains_this_entry[context_bb_pair]=true;
                    }
                    if(!backward_worklist_contains_this_entry[context_bb_pair])
                    {
                        backward_worklist.push(context_bb_pair);
                        backward_worklist_contains_this_entry[context_bb_pair]=true;
                    }
                    // forward_worklist.push(context_bb_pair);
                    // backward_worklist.push(context_bb_pair);
                }
            }

        }
        // errs()<<WHITEB<<"\nBB_OUT"<<CS_BB_OUT[current_pair].first[0]<<RST;
    }
    errs()<<GREENB"\nNothing left to process in forward direction."<<RST;
}

template <class F,class B>
F Analysis<F,B>::NormalFlowFunctionForward(pair<int,BasicBlock*> current_pair_of_context_label_and_bb)
{
    // outs() << "AYUSH" << "\n";
    BasicBlock &b=*(current_pair_of_context_label_and_bb.second);
    // setProcessingContextLabel(current_pair_of_context_label_and_bb.first);
    F prev=CS_BB_IN[current_pair_of_context_label_and_bb].first;
    //traverse a basic block in forward direction
    for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
    {
        outs() << "\n===================================FORWARD===========================================" << "\n";
        outs() << "INSTRUCTION: " << *inst << "\n";
        // errs()<<"\n"<<*inst;
        // IN[&(*inst)].first=prev;//compute IN from previous OUT-value
        outs() << "IN: ";
        printDataFlowValuesForward(prev);
        setForwardComponentAtInOfThisInstruction(&(*inst),prev);//compute IN from previous OUT-value
        // OUT[&(*inst)].first=computeOutFromIn(*inst);
        F new_prev = computeOutFromIn(*inst);
        // outs() << new_prev.size();
        // for(auto p : new_prev){
        //     outs() << "VALUE IS: ";
        //     outs() << "(";
        //     outs() << p.first << "=" << p.second;
        //     outs() << "),";
        // }
        setForwardComponentAtOutOfThisInstruction(&(*inst),new_prev);
        outs() << "OUT: ";
        printDataFlowValuesForward(new_prev);
        // prev=OUT[&(*inst)].first;
        prev=getForwardComponentAtOutOfThisInstruction(*inst);
        outs() << "===================================FORWARD===========================================" << "\n";
    }
    return prev;
}

template <class F,class B>
int Analysis<F,B>::check_if_context_already_exists(pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object)
{
    // errs()<<CYANB"\nINSIDE METHOD CHECK FOR EXISTING CONTEXT. Total existing contexts="<<ProcedureContext.size()<<RST;
    if(std::is_same<B, NoAnalysisType>::value)
    {
        //forward only
        // F some_irrelevant_local_value=getBoundaryInformationForward();
        for(auto set_itr:ProcedureContext)
        {
            pair<Function*,pair<pair<F,B>,pair<F,B>>> current_object= context_label_to_context_object_map[set_itr];
            F new_context_values=new_context_object.second.first.first;
            F current_context_values=current_object.second.first.first;
            if(new_context_object.first==current_object.first&&EqualDataFlowValuesForward(new_context_values,current_context_values))
            {
                return set_itr;
            }
        }
    }
    else if(std::is_same<F, NoAnalysisType>::value)
    {
        //backward only
        // B some_irrelevant_local_value=getBoundaryInformationBackward(); 
        for(auto set_itr:ProcedureContext)
        {
            pair<Function*,pair<pair<F,B>,pair<F,B>>> current_object= context_label_to_context_object_map[set_itr];
            // errs()<<YELLOWB<<"\nExisting val:"<<current_object.second.first.second[0]<<" Curr Val:"<<new_context_object.second.first.second[0]<<" Exis func:"<<current_object.first->getName()<<" Curr Func:"<<new_context_object.first->getName()<<" Set itr:"<<set_itr;
            B new_context_values= new_context_object.second.first.second;
            B current_context_values= current_object.second.first.second;
            if(new_context_object.first==current_object.first&&EqualDataFlowValuesBackward(new_context_values,current_context_values))
            {
                errs()<<YELLOWB<<"\nMatching Context found!";
                return set_itr;
            }
        }
        errs()<<REDB<<"\nNO Matching Context found!";
    }
    else
    {
        //bidirectional
        // F some_irrelevant_local_value_forward=getBoundaryInformationForward();
        // B some_irrelevant_local_value_backward=getBoundaryInformationBackward();
        int i=0;
        for(auto set_itr:ProcedureContext)
        {
            // errs()<<GREENB"\n"<<set_itr;
            pair<Function*,pair<pair<F,B>,pair<F,B>>> current_object= context_label_to_context_object_map[set_itr];
            
            F new_context_values_forward = new_context_object.second.first.first;
            F current_context_values_forward = current_object.second.first.first;
            B new_context_values_backward = new_context_object.second.first.second;
            B current_context_values_backward = current_object.second.first.second;
            
            // errs()<<"\nNewForward";
            // printDataFlowValuesForward(new_context_values_forward);
            // errs()<<"\nCurrentForward";
            // printDataFlowValuesForward(current_context_values_forward);
            errs()<<"\n-------------------------------";
            if(new_context_object.first==current_object.first&&EqualDataFlowValuesBackward(new_context_values_backward,current_context_values_backward)&&EqualDataFlowValuesForward(new_context_values_forward,current_context_values_forward))
            {
                errs()<<YELLOWB<<"\nMatching Context found!";
                return set_itr;
            }
        }
        errs()<<REDB<<"\nNO Matching Context found!"; 
    }
    return 0;
}

template <class F,class B>
void Analysis<F,B>::printWorklistMaps()
{
	errs()<<REDB<<"\n-----------------------------------------";
	errs()<<WHITEB;
	for(auto x:backward_worklist_contains_this_entry)
	{
	    errs()<<"\n";
	    errs()<<x.first.first<<" ";
	    BasicBlock *bb=x.first.second;
	    bb->printAsOperand(errs(),false);
	    errs()<<x.second;
	}
	errs()<<REDB<<"\n-----------------------------------------";
	errs()<<RST;
}

template <class F,class B>
void Analysis<F,B>::doAnalysisBackward()
{
    while(!backward_worklist.empty())//step 2
    {
        //step 3 and 4
        pair<int,BasicBlock*> current_pair=backward_worklist.top();
        int current_context_label;
        BasicBlock *bb;
        current_context_label=backward_worklist.top().first;
        setProcessingContextLabel(current_context_label);
        bb=backward_worklist.top().second;
        backward_worklist.pop();
        backward_worklist_contains_this_entry[make_pair(current_context_label,bb)]=false;

        BasicBlock &b=*bb;
        Function *f=context_label_to_context_object_map[current_context_label].first;
        Function &function=*f;
        errs()<<BLUEB"\nContext Label: "<<current_context_label<<" BasicBlock: ";
        b.printAsOperand(errs(),false);
        errs()<<" Function Name: "<<f->getName()<<" WS:"<<backward_worklist.size()<<RST;
        
        //step 5
        if(bb!=(&function.back()))
        {
            errs()<<"\nNon-Exit Block";
            //step 6
            CS_BB_OUT[current_pair].second=getInitialisationValueBackward();
            //step 7 and 8
            for(auto succ_bb:successors(bb))
            {
                errs()<<"\nSucc BB: ";
                succ_bb->printAsOperand(errs(),false);
                errs() << "\t"; //Label of Basic block
                CS_BB_OUT[current_pair].second=performMeetBackward(CS_BB_OUT[current_pair].second,CS_BB_IN[make_pair(current_pair.first,succ_bb)].second);
            }
        }
        else
        {
            errs()<<"\nExit Block";
            //Out value of this node is same as INFLOW value
            CS_BB_OUT[current_pair].second=getBackwardInflowForThisContext(current_context_label);
            // CS_BB_OUT[current_pair].second=context_label_to_context_object_map[current_context_label].second.first.second;
        }
        
        //step 9
        F a1=CS_BB_IN[current_pair].first;
        B d1=CS_BB_OUT[current_pair].second;

        B previous_value_at_in_of_this_node=CS_BB_IN[current_pair].second;
        //step 10
        bool contains_a_method_call=false;
        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        {
            Instruction &I=*inst;
            if ( CallInst *ci = dyn_cast<CallInst>(&I))
            {
                Function* target_function= ci->getCalledFunction();
                // errs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
                // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
                // {
                //     continue;//this is an inbuilt function so doesnt need to be processed.
                // }
                if((*target_function).size()==0 || isAnIgnorableDebugInstruction(&I))
                {
                    continue;//this is an inbuilt function so doesnt need to be processed.
                }
                // contains_a_method_call=true;
                contains_a_method_call=true;
                errs()<<REDB"\nMethod Call found in basic block: "<<inst->getParent()->getName()<<RST;
                break;
            }
        }
        // errs()<<MAGENTAB"\nContains a method call: "<<contains_a_method_call<<RST;
        if(contains_a_method_call)
        {
            B prev=CS_BB_OUT[current_pair].second;
            
            //step 11
            for(auto inst=b.rbegin();inst!=b.rend();inst++)
            {
                Instruction &I=*inst;
                if ( CallInst *ci = dyn_cast<CallInst>(&I))
                {

                    Function* target_function= ci->getCalledFunction();
                    // errs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
                    // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
                    // {
                    //     continue;
                    // }
                    if((*target_function).size()==0||isAnIgnorableDebugInstruction(&I))
                    {
                        continue;
                    }
                    
                    errs()<<"\n"<<*inst;
                    
                    /*
                    At the call instruction, the value at OUT should be splitted into two components:
                    1) Purely Global and 2) Mixed.
                    The purely global component is given to the end of callee.
                    */
                    //step 12
                    pair<F,B> inflow_pair=CallInflowFunction(current_context_label,target_function,bb,a1,d1);
                    F a2=inflow_pair.first;
                    B d2=inflow_pair.second;

                    // F new_inflow_forward;
                    // B new_inflow_backward=getPurelyGlobalComponentBackward(prev);//prev;
                    F new_outflow_forward;
                    B new_outflow_backward;
                    
                    
                    
                    //step 13
                    // pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(new_inflow_forward,new_inflow_backward),make_pair(new_outflow_forward,new_outflow_backward)));
                    pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(a2,d2),make_pair(new_outflow_forward,new_outflow_backward)));

                    //===========================================================
                    // OUT[&(*inst)].second=prev;//compute OUT from previous IN-value
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),prev);//compute OUT from previous IN-value
                    //===========================================================
                    // errs()<<WHITEB<<"\nINFLOW:"<<prev[0];
                    int matching_context_label=0;
                    matching_context_label=check_if_context_already_exists(new_context_object);
                    outs() << "MATCHING CONTEXT LABEL: ";
                    outs() << matching_context_label << "\n";
                    if(matching_context_label>0)//step 15
                    {
                        errs()<<BLUEB"\nExisting context found!"<<matching_context_label<<RST;
                        //step 14
                        pair<int,Instruction*>mypair=make_pair(current_context_label,&(*inst));
                        context_transition_graph[mypair]=matching_context_label;

                        //step 16 and 17
                        F a3=getForwardOutflowForThisContext(matching_context_label);
                        B d3=getBackwardOutflowForThisContext(matching_context_label);
                        // F a3=context_label_to_context_object_map[matching_context_label].second.second.first;
                        // B d3=context_label_to_context_object_map[matching_context_label].second.second.second;
                        
                        pair<F,B> outflow_pair=CallOutflowFunction(current_context_label,target_function,bb,a3,d3,a1,d1);
                        F a4=outflow_pair.first;
                        B value_to_be_meet_with_prev_in=outflow_pair.second;
                        
                        // new_outflow_backward=getBackwardOutflowForThisContext(matching_context_label);
                        // errs()<<WHITEB<<"\nNew Outflow before meet:";
                        // // <<new_outflow_backward[0]<<RST;
                        // for(auto x:value_to_be_meet_with_prev_in)
                        // {
                        //     errs()<<"\t"<<x->getName();
                        // }
                        // errs()<<RST;
                        
                        

                        //===========================================================
                        //step 18 and 19
                        
                        /*
                        At the call instruction, the value at OUT should be splitted into two components.
                        The purely global component is given to the callee while the mixed component is propagated
                        to IN of this instruction after executing computeINfromOUT() on it.
                        */
                        
                        
                        // B value_to_be_propagated_to_in_of_instruction=computeInFromOut(*inst);//d5
                        
                        /*
                        At the IN of this instruction, the value from START of callee procedure is to be merged
                        with the local(mixed) value propagated from OUT. Note that this merging "isn't" 
                        exactly (necessarily) the meet between these two values.
                        */
                        
                        // B merged_value_at_in_of_instruction=getCombinedValuesAtCallBackward(new_outflow_backward,value_to_be_propagated_to_in_of_instruction);//combining d4 and d5

                        
                        /*
                        As explained in ip-vasco,pdf, we need to perform meet with the original value of IN
                        of this instruction to avoid the oscillation problem.
                        */
                        
                        setBackwardComponentAtInOfThisInstruction(&(*inst),performMeetBackward(value_to_be_meet_with_prev_in,getBackwardComponentAtInOfThisInstruction((*inst))));
                        
                        prev=getBackwardComponentAtInOfThisInstruction((*inst));
                        // errs()<<WHITEB<<"\nNew Inflow after meet:"<<prev[0]<<RST;
                        
                        //===========================================================
                    }
                    else//step 20
                    {
                        errs() <<REDB"\n----------------------------------------------\n";
                        errs() << MAGENTAB"CREATING A NEW CONTEXT!!!!!!!" << "\n";
                        //creating a new context
                        INIT_CONTEXT(new_context_object);//step 21

                        pair<int,Instruction*>mypair=make_pair(current_context_label,&(*inst));
                        //step 14
                        context_transition_graph[mypair]=context_label_counter;
                        errs() << "LABEL:- " << context_label_counter << "\n";
                        errs() << "INSTRUCTION:- " << *inst << "\n";
                        errs() <<REDB"\n----------------------------------------------\n";
                        return;
                        //===========================================================
                        // IN[&(*inst)].second=computeInFromOut(*inst);
                        // prev=IN[&(*inst)].second;
                        //===========================================================
                    }
                }
                else
                {
                    // errs() << CYANB <<"\n"<<*inst << "AYUSH";
                    // OUT[&(*inst)].second=prev;//compute OUT from previous IN-value
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),prev);//compute OUT from previous IN-value
                    // IN[&(*inst)].second=computeInFromOut(*inst);
                    setBackwardComponentAtInOfThisInstruction(&(*inst),computeInFromOut(*inst));
                    // prev=IN[&(*inst)].second;
                    prev=getBackwardComponentAtInOfThisInstruction((*inst));
                }
            }
            CS_BB_IN[current_pair].second=prev;
        }
        else//step 22
        {
            //step 23
            //NormalFlowFunction
            CS_BB_IN[current_pair].second=NormalFlowFunctionBackward(current_pair);
            
        }
        bool changed=false;
        if(!EqualDataFlowValuesBackward(previous_value_at_in_of_this_node,CS_BB_IN[current_pair].second))
        {
            changed=true;
        }
        if(changed)//step 24
        {
            //not yet converged
            for(auto pred_bb:predecessors(bb))//step 25
            {
                errs()<<"\nPushing Pred BB: ";
                pred_bb->printAsOperand(errs(),false);
                errs() << "\t"; //Label of Basic block

                //step 26
                if(!forward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)])
                {
                    forward_worklist.push(make_pair(current_context_label,pred_bb));
                    forward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)]=true;
                }
                if(!backward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)])
                {
                    backward_worklist.push(make_pair(current_context_label,pred_bb));
                    backward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)]=true;
                }
                // forward_worklist.push(make_pair(current_context_label,pred_bb));
                // backward_worklist.push(make_pair(current_context_label,pred_bb));  
            }

        }
        
        if(bb==&function.getEntryBlock())//step 27
        {
            //first node/start node
            
            //step 28
            // errs()<<WHITEB<<"\nBACKWARD OUTFLOW VALUE(before):"<<context_label_to_context_object_map[current_context_label].second.second.second[0];
            // errs()<<WHITEB<<"\nSetting outflow!";
            // for(auto x:CS_BB_IN[current_pair].second)
            // {
            //     errs()<<"\t"<<x->getName();
            // }
            // errs()<<RST;
            setBackwardOutflowForThisContext(current_context_label,getPurelyGlobalComponentBackward(CS_BB_IN[current_pair].second));//setting backward outflow
            // context_label_to_context_object_map[current_context_label].second.second.second=getPurelyGlobalComponentBackward(CS_BB_IN[current_pair].second);//setting backward outflow
            
            // errs()<<WHITEB<<"\nBACKWARD OUTFLOW VALUE(after):"<<context_label_to_context_object_map[current_context_label].second.second.second[0];
            
            for(auto context_inst_pairs:context_transition_graph)//step 29
            {
                if(context_inst_pairs.second==current_context_label)//matching the called function
                {
                    //step 30
                    
                    errs()<<CYANB<<"\ncaller context label:"<<context_inst_pairs.first.first<<RST;
                    
                    BasicBlock *bb=context_inst_pairs.first.second->getParent();
                    pair<int,BasicBlock*>context_bb_pair=make_pair(context_inst_pairs.first.first,bb);
                    if(!forward_worklist_contains_this_entry[context_bb_pair])
                    {
                        forward_worklist.push(context_bb_pair);
                        forward_worklist_contains_this_entry[context_bb_pair]=true;
                    }
                    if(!backward_worklist_contains_this_entry[context_bb_pair])
                    {
                        backward_worklist.push(context_bb_pair);
                        backward_worklist_contains_this_entry[context_bb_pair]=true;
                    }
                    // forward_worklist.push(context_bb_pair);
                    // backward_worklist.push(context_bb_pair);
                    
                }
            }

        }
        
        // errs()<<WHITEB<<"\nBB_IN"<<CS_BB_IN[current_pair].second[0]<<RST;
    }
    // else
    // {
        //worklist is empty
        errs()<<GREENB"\nNothing left to process in backward direction."<<RST;
    // }
}

template <class F,class B>
B Analysis<F,B>::NormalFlowFunctionBackward(pair<int,BasicBlock*> current_pair_of_context_label_and_bb)
{
    outs() << "AYUSH" << "\n";
    BasicBlock &b=*(current_pair_of_context_label_and_bb.second);
    B prev=CS_BB_OUT[current_pair_of_context_label_and_bb].second;
    F prev_f = CS_BB_OUT[current_pair_of_context_label_and_bb].first;
    //traverse a basic block in backward direction
    // for(auto inst=b.rbegin();inst!=b.rend();inst++)
    for(auto inst=&*(b.rbegin());inst!=NULL;inst=inst->getPrevNonDebugInstruction())
    {
        outs() << "===================================BACKWARD===========================================" << "\n";
        outs() << "INSTRUCTION: " << *inst << "\n";
        // errs()<<"\n"<<*inst;
        // OUT[&(*inst)].second=prev;//compute OUT from previous IN-value
        setBackwardComponentAtOutOfThisInstruction(&(*inst),prev);//compute OUT from previous IN-value
        // IN[&(*inst)].second=computeInFromOut(*inst);
        outs() << "OUT: ";
        outs() << "\n";
        printDataFlowValuesForward(prev_f);
        printDataFlowValuesBackward(prev);
        outs() << "\n";
        B new_dfv = computeInFromOut(*inst);
        setBackwardComponentAtInOfThisInstruction(&(*inst),new_dfv);
        // prev=IN[&(*inst)].second;
        outs() << "IN: " << "\n";
        printDataFlowValuesForward(prev_f);
        printDataFlowValuesBackward(new_dfv);
        outs() << "\n";
        prev=getBackwardComponentAtInOfThisInstruction(*inst);
        // errs()<<MAGENTAB"\nnormal prev:";
        // for(auto x:prev)
        // {
        //     errs()<<"\t"<<x->getName();
        // }
        // errs()<<"\n";
        // errs()<<RST;
        outs() << "===================================BACKWARD===========================================" << "\n";
    }
    return prev;
}



template <class F,class B>
void Analysis<F,B>::performSplittingBB(Function &function)
{
    for(BasicBlock *BB:inverse_post_order(&function.back()))
    {
        errs()<<GREENB;
        BasicBlock &b=*BB;
        errs()<<"\nBasicBlock: ";
        b.printAsOperand(errs(),false);
        errs()<<WHITEB;
        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        {
            errs()<<"\n";
            errs()<<(*inst);
        }
    }






//================================================================================================    
    // int flag=0;
    // for(BasicBlock *BB:post_order(&function.getEntryBlock()))
    // {
    //     // errs()<<GREENB;
    //     BasicBlock &b=*BB;
    //     // errs()<<"\nBasicBlock: ";
    //     // b.printAsOperand(errs(),false);
    //     // errs()<<RST;
    //     // errs()<<CYANB;
    //     Instruction *I=&(*(b.begin()));
    //     for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
    //     {
    //         // errs()<<"\n";
    //         // errs()<<(*inst);
    //         Instruction &ins=*inst;
    //         if (isa<CallInst>(ins))
    //         {
    //             I=&(*inst);
    //             b.splitBasicBlock(I);
    //             flag=1;
    //             break;
    //         }
    //     }
    //     if(flag==1)
    //     {
    //         errs()<<MAGENTAB<<"\nSplitting On:"<<*I;
    //         flag=0;
    //         // break;
    //     }
    // }
    //================================================================================================
    errs()<<CYANB"\n----------------------------------------------------------------\n";
    int flag=0;
    Instruction *I=NULL;
    bool previousInstructionWasSplitted=false;
    bool previousInstructionWasCallInstruction=false;
    map<Instruction *,bool>isSplittedOnThisInstruction;
    for(BasicBlock *BB:inverse_post_order(&function.back()))
    {
        // errs()<<GREENB;
        BasicBlock &b=*BB;
        // if(b.size()==2)
        //     continue;
        // errs()<<"\nBasicBlock: ";
        // b.printAsOperand(errs(),false);
        // errs()<<WHITEB;
        previousInstructionWasSplitted=true;
        previousInstructionWasCallInstruction=false;
        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        {
            I=&(*inst);
            if(inst==&*(b.begin()))
            {
                if (isa<CallInst>(*I))
                {
                    CallInst *ci = dyn_cast<CallInst>(I);
            
                    Function* target_function= ci->getCalledFunction();
                    // errs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
                    // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
                    // {
                    //     continue;//this is an inbuilt function so doesn't need to be processed.
                    // }
                    if((*target_function).size()==0 || isAnIgnorableDebugInstruction(I))
                    {
                        continue;//this is an inbuilt function so doesn't need to be processed.
                    }
                    isSplittedOnThisInstruction[I]=false;
                    previousInstructionWasCallInstruction=true;
                    previousInstructionWasSplitted=true;//creating a false positive
                }
                continue;    
            }
            // errs()<<"\n";
            // errs()<<(*inst);
            if(isa<BranchInst>(*I))
            {
                
            }
            else if (isa<CallInst>(*I))
            {
                CallInst *ci = dyn_cast<CallInst>(I);
                
        
                Function* target_function= ci->getCalledFunction();
                // errs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
                // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
                // {
                //     continue;
                // }
                if((*target_function).size()==0 || isAnIgnorableDebugInstruction(I))
                {
                    continue;
                }
                isSplittedOnThisInstruction[I]=true;
                previousInstructionWasCallInstruction=true;
                previousInstructionWasSplitted=true;
            }
            else
            {
                if(previousInstructionWasSplitted)
                {
                    if(previousInstructionWasCallInstruction)
                    {
                        isSplittedOnThisInstruction[I]=1;
                        previousInstructionWasSplitted=true;
                    }
                    else
                    {
                        previousInstructionWasSplitted=false;
                    }
                }
                else
                {
                    //do nothing
                }
                previousInstructionWasCallInstruction=false;
            }
        }
    }
    BasicBlock *containingBB;
    
    for(auto split_here:isSplittedOnThisInstruction)
    {
        // errs()<<"\n";
        // errs()<<*(split_here.first);
        if(split_here.second==false)//no splitting to be done
            continue;
        containingBB=split_here.first->getParent();
        containingBB->splitBasicBlock(split_here.first);
    }
    // errs()<<RST;
errs()<<CYANB"\n----------------------------------------------------------------\n";
//================================================================================================
   /* errs()<<MAGENTAB"\n----------------------------------------------------------------\n";
    int flag=0;
    map<Instruction *,bool>isSplittedOnThisInstruction;
    for(BasicBlock *BB:inverse_post_order(&function.back()))
    {
        BasicBlock &b=*BB;
        errs()<<GREENB;
        errs()<<"\nBasicBlock: ";
        b.printAsOperand(errs(),false);
        errs()<<RST;
        // errs()<<CYANB;
        Instruction *I=NULL;
        BasicBlock *bb=BB;
        while(1)
        {
                
            auto inst=(*bb).begin();
            while(1)//(auto inst=b.begin();inst!=b.end();inst++)
            {
                errs()<<"\n"<<YELLOWB;
                errs()<<(*inst);
                errs()<<RST;
                Instruction &ins=*inst;
                I=&(*inst);
                if (isa<CallInst>(ins)&&isSplittedOnThisInstruction[I]==false)
                {
                    isSplittedOnThisInstruction[I]=true;
                    bb=(*bb).splitBasicBlock(I);
                    flag=1;
                    break;
                }
                inst++;
                if(inst==(*bb).end())
                    break;
            }
            if(flag==1)
            {
                errs()<<MAGENTAB<<"\nSplitting On:"<<*I<<RST;
                flag=0;
                // break;
            }
            else
            {
                break;
            }
        }
    }
    errs()<<MAGENTAB"\n----------------------------------------------------------------\n";
    */
    for(BasicBlock *BB:inverse_post_order(&function.back()))
    {
        errs()<<YELLOWB;
        BasicBlock &b=*BB;
        errs()<<"\nBasicBlock: ";
        b.printAsOperand(errs(),false);
        errs()<<WHITEB;
        
        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        {
            errs()<<"\n";
            errs()<<(*inst);
        }
        // errs()<<"\nFetching next inst!";
        // Instruction* inst=&*(b.begin());
        // while(inst!=NULL)
        // {
        //     errs()<<"\n";
        //     errs()<<(*inst);
        //     inst=inst->getNextNonDebugInstruction();
        // }
        // for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        // {
        //     errs()<<"\n";
        //     errs()<<(*inst);
        // }
    }
    errs()<<RST;
    
}

template <class F,class B>
void Analysis<F,B>::drawSuperGraph(Module &M)
{
    filebuf fb;
    fb.open ("../testcases/supergraph/supergraph1.dot",ios::out);
    ostream os(&fb);
    // os << "Test sentence\n";
    string str;
    llvm::raw_string_ostream ss(str);
    string str1;
    llvm::raw_string_ostream ss1(str1);
    int cnt=0;
    os<< "digraph{\ngraph[fontsize=40];\n";
    for(Function &function:M)
    {
        os << "subgraph cluster_"<<cnt++<<"{\n";
        os << "style=filled;\ncolor=lightgrey;\nnode [style=filled,fillcolor=white,color=black,fontsize=40];\n";
        for(BasicBlock *BB:inverse_post_order(&function.back()))
        {
            BasicBlock &b=*BB;
            os << "BB";
            b.printAsOperand(ss1);
            os<<str1;
            string bbname=b.getName().str();
            os << " [label = \"BasicBlock "<<bbname<<":\\l";
            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
            {
                Instruction &I=*inst;
                // os<<I<<"\n";
                
                ss << I;
                os << ss.str() << "\n";
                str="";
            }
            os<<"\"];\n";
        }
        string procname=function.getName().str();
        os <<"label=" << "\"Procedure : "<<procname<<"\";\n";
        os << "}\n";
        
    }
    os<< "label="<<"supergraph\n"<<"}\n";
    fb.close();
}

template <class F,class B>
void Analysis<F,B>::printContext() {
    outs() << "\n";
    for(auto val : context_label_to_context_object_map) {
        outs() << "LABEL: " << val.first << " ";
        outs() << "FUNCTION NAME: " << val.second.first->getName().str() << " ";
        outs() << "INFLOW VALUE: " << " ";
        // cout << "< ";
        printDataFlowValuesForward(val.second.second.first.first);
        // cout << ",";
        printDataFlowValuesBackward(val.second.second.first.second);
        // cout << " >";
        outs() << "OUTFLOW VALUE: " << " ";
        printDataFlowValuesForward(val.second.second.second.first);
        printDataFlowValuesBackward(val.second.second.second.second);
        outs() << "\n";
    }
}






#endif

