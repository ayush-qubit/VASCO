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
#include <unordered_map>
#include <unordered_set>
#include <ostream>

#include "Context.h"

#define RST "\033[0;m"
#define REDB "\033[1;91m"
#define WHITEB "\033[1;97m"

using namespace llvm;
using namespace std;
enum NoAnalysisType {NoAnalyisInThisDirection};

class HashFunction{
public:
    long double operator () (const pair<int,BasicBlock *> &P) const{
        std::string s1 = std::to_string(P.first);
        std::string s2 = std::to_string((long)P.second);
        return stold(s1+s2);
    }
    long double operator () (const pair<int, Instruction *> &P) const{
        std::string s1 = std::to_string(P.first);
        std::string s2 = std::to_string((long)P.second);
        return stold(s1+s2);
    }
};



// TODO can replace all contexts with context label counter
template<class F,class B>
class Analysis
{
    private:
    Module *current_module;
    int context_label_counter;
    int current_analysis_direction; //0:initial pass, 1:forward, 2:backward
    int processing_context_label;
//    std::unordered_map<pair<int,Instruction*>,pair<F,B>,HashFunction> IN, OUT;
    std::unordered_map<int,unordered_map<Instruction *,pair<F,B>>> IN, OUT;
    std::unordered_map<int,bool> isFree;
    std::string direction;

    //mapping from context label to context object
//    map<int,pair<Function*,pair<pair<F,B>,pair<F,B>>>>context_label_to_context_object_map;
    unordered_map<int,Context<F,B> *> context_label_to_context_object_map;
    
    //mapping from context object to context label
    //mapping from function to  pair<inflow,outflow>
    //inflow and outflow are themselves pairs of forward and backward component values.
    //The forward and backward components are themselves pairs of G,L dataflow values.

//    map<pair<Function*,pair<pair<F,B>,pair<F,B>>>,int>context_object_to_context_label_map;
//    map<Context<F,B>,int> context_object_to_context_label_map;
    bool debug;

    void printLine(int);
	
    protected:
    
    //List of contexts
    unordered_set<int> ProcedureContext;

    // mapping from (context label,basic block) to (forward and backward) data flow values
//    unordered_map<pair<int,BasicBlock*>,std::pair<F,B>,HashFunction> CS_BB_IN, CS_BB_OUT;
    unordered_map<int,unordered_map<BasicBlock *,std::pair<F,B>>> CS_BB_IN, CS_BB_OUT;

    // worklist of (context label,basic block) for both directions of analysis
    stack<pair<int,BasicBlock*>> backward_worklist, forward_worklist;
    
    // mapping to check which entries are already part of the worklist (key,value)
    unordered_set<pair<int,BasicBlock*>,HashFunction> isPresentForward;
    unordered_set<pair<int,BasicBlock*>,HashFunction> isPresentBackward;

    // mapping from (context label,call site) to target context label
    unordered_map<pair<int,Instruction*>,int,HashFunction> context_transition_graph; //graph
    raw_ostream *out;
    public:
    Analysis(bool);
    Analysis(bool,string);
	int getContextLabelCounter();
    void setContextLabelCounter(int);
    int getCurrentAnalysisDirection();
    void setCurrentAnalysisDirection(int);
    int getProcessingContextLabel();
    void setProcessingContextLabel(int);
    int getNumberOfContexts();
    
    void doAnalysis(Module &M);
//    void INIT_CONTEXT(pair<Function*,pair<pair<F,B>,pair<F,B>>> context_object);
//    void INIT_CONTEXT(Context<F,B>);
    void INIT_CONTEXT(llvm::Function *, std::pair<F,B>, std::pair<F,B>);
    void doAnalysisForward();
//    void doAnalysisBackward();
    F NormalFlowFunctionForward(pair<int,BasicBlock*>);
//    B NormalFlowFunctionBackward(pair<int,BasicBlock*>);
//    int check_if_context_already_exists(pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object);
    int check_if_context_already_exists(llvm::Function *, pair<F,B>, pair<F,B>);
    void drawSuperGraph(Module &M);
    void printWorklistMaps();
    // bool isAnIgnorableDebugInstruction(std::string mainStr, std::string toMatch);
    bool isAnIgnorableDebugInstruction(Instruction *);
    void performSplittingBB(Function &f);
    void freeMemory(int);

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

    pair<F,B> getIn(int, llvm::BasicBlock *);
    pair<F,B> getOut(int, llvm::BasicBlock *);
    void setForwardIn(int, llvm::BasicBlock *, F);
    void setForwardOut(int, llvm::BasicBlock *, F);
    void setBackwardIn(int, llvm::BasicBlock *, B);
    void setBackwardOut(int, llvm::BasicBlock *, B);


    F getForwardInflowForThisContext(int);
    B getBackwardInflowForThisContext(int);
    F getForwardOutflowForThisContext(int);
    B getBackwardOutflowForThisContext(int);

    void setForwardInflowForThisContext(int,F);
    void setBackwardInflowForThisContext(int,B);
    void setForwardOutflowForThisContext(int,F);
    void setBackwardOutflowForThisContext(int,B);

    bool forward_worklist_contains_this_entry(int, llvm::BasicBlock *);
    bool backward_worklist_contains_this_entry(int, llvm::BasicBlock *);
    void set_forward_worklist_contains_this_entry(int, llvm::BasicBlock *);
    void set_backward_worklist_contains_this_entry(int, llvm::BasicBlock *);
    void delete_forward_worklist_contains_this_entry(int, llvm::BasicBlock *);
    void delete_backward_worklist_contains_this_entry(int, llvm::BasicBlock *);

    void printModule(Module &M);

    Function* getFunctionAssociatedWithThisContext(int);
    void setFunctionAssociatedWithThisContext(int,Function*);

    void printContext();

    virtual pair<F,B> CallInflowFunction(int,Function*,BasicBlock*,F,B);
    virtual pair<F,B> CallOutflowFunction(int,Function*,BasicBlock*,F,B,F,B);


    virtual void printResults(){}
	
	virtual F computeOutFromIn(Instruction &I);
    virtual F getBoundaryInformationForward();//{}
    virtual F getInitialisationValueForward();//{}
    virtual F performMeetForward(F d1,F d2);//{}
    virtual bool EqualDataFlowValuesForward(F d1,F d2);//{}
    virtual F getPurelyLocalComponentForward(F dfv);
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

    void printInOutMaps();
    virtual void printForwardWorklist(){}
    virtual void printBackwardWorklist(){}
    


};

//========================================================================================
template<class F,class B>
void Analysis<F,B>::printLine(int label) {
    std::string Str = "\n===================================[ FORWARD-"+ to_string(label)+" ]===========================================\n";
    llvm::outs() << Str;
}


template<class F,class B>
Analysis<F,B>::Analysis(bool debug){
    current_module=nullptr;
    context_label_counter=-1;
    this->debug = debug;
    this->out = &llvm::outs();
    this->direction = "";
}

template<class F,class B>
Analysis<F,B>::Analysis(bool debug,string fileName){
    current_module=nullptr;
    context_label_counter=-1;
    this->debug = debug;
    freopen(fileName.c_str(),"w",stdout);
    this->out = &llvm::outs();
    this->direction = "";
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

template <class F,class B>
B Analysis<F,B>::computeInFromOut(Instruction &I)
{
    llvm::outs() <<"\nThis function computeInFromOut() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::computeOutFromIn(Instruction &I)
{
    llvm::outs()<<"\nThis function computeOutFromIn() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getBoundaryInformationForward(){
    llvm::outs()<<"\nThis function getBoundaryInformationForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
B Analysis<F,B>::getBoundaryInformationBackward(){
    llvm::outs()<<"\nThis function getBoundaryInformationBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getInitialisationValueForward(){
    llvm::outs()<<"\nThis function getInitialisationValueForward() has not been implemented. EXITING !!\n";
    exit(-1);   
}

template <class F,class B>
B Analysis<F,B>::getInitialisationValueBackward(){
    llvm::outs()<<"\nThis function getInitialisationValueBackward() has not been implemented. EXITING !!\n";
    exit(-1);   
}

template <class F,class B>
F Analysis<F,B>::performMeetForward(F d1,F d2){
    llvm::outs()<<"\nThis function performMeetForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
B Analysis<F,B>::performMeetBackward(B d1,B d2){
    llvm::outs()<<"\nThis function performMeetBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
bool Analysis<F,B>::EqualDataFlowValuesForward(F d1,F d2){
    llvm::outs()<<"\nThis function EqualDataFlowValuesForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
bool Analysis<F,B>::EqualDataFlowValuesBackward(B d1,B d2){
    llvm::outs()<<"\nThis function EqualDataFlowValuesBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
B Analysis<F,B>::getPurelyGlobalComponentBackward(B dfv)
{
    llvm::outs()<<"\nThis function getPurelyGlobalComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
F Analysis<F,B>::getPurelyGlobalComponentForward(F dfv)
{
    llvm::outs()<<"\nThis function getPurelyGlobalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getPurelyLocalComponentForward(F dfv)
{
    llvm::outs()<<"\nThis function getPurelyLocalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}
/*
template <class F,class B>
B Analysis<F,B>::getPurelyLocalComponentBackward(B dfv)
{
    llvm::outs()<<"\nThis function getPurelyLocalComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}

template <class F,class B>
F Analysis<F,B>::getPurelyLocalComponentForward(F dfv)
{
    llvm::outs()<<"\nThis function getPurelyLocalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);  
}



template <class F,class B>
B Analysis<F,B>::getMixedComponentBackward(B dfv)
{
    llvm::outs()<<"\nThis function getMixedComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getMixedComponentForward(F dfv)
{
    llvm::outs()<<"\nThis function getMixedComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
B Analysis<F,B>::getCombinedValuesAtCallBackward(B dfv1,B dfv2)
{
    llvm::outs()<<"\nThis function getCombinedValuesAtCallBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getCombinedValuesAtCallForward(F dfv1,F dfv2)
{
    llvm::outs()<<"\nThis function getCombinedValuesAtCallForward() has not been implemented. EXITING !!\n";
    exit(-1);
}
*/

//==========================Checking for entry==============================

template<class F,class B>
bool Analysis<F,B>::forward_worklist_contains_this_entry(int label, llvm::BasicBlock *BB) {
    if(isPresentForward.find({label,BB}) != isPresentForward.end()){
        return true;
    }
    return false;
}
template<class F,class B>
bool Analysis<F,B>::backward_worklist_contains_this_entry(int label, llvm::BasicBlock *BB) {
    if(isPresentBackward.find({label,BB}) != isPresentBackward.end()){
        return true;
    }
    return false;
}

template<class F,class B>
void Analysis<F,B>::set_forward_worklist_contains_this_entry(int label, llvm::BasicBlock *BB) {
    isPresentForward.insert({label,BB});
}
template<class F,class B>
void Analysis<F,B>::set_backward_worklist_contains_this_entry(int label, llvm::BasicBlock *BB) {
    isPresentBackward.insert({label,BB});
}

template<class F,class B>
void Analysis<F,B>::delete_forward_worklist_contains_this_entry(int label, llvm::BasicBlock *BB) {
    isPresentForward.erase({label,BB});
}
template<class F,class B>
void Analysis<F,B>::delete_backward_worklist_contains_this_entry(int label, llvm::BasicBlock *BB) {
    isPresentBackward.erase({label,BB});
}



//========================================================================================

template<class F,class B>
void Analysis<F,B>::printModule(Module &M){
    llvm::outs() << "--------------------------------------------" << "\n";
    for(Function& Func : M){
        llvm::outs() << "FUNCTION NAME: ";
        llvm::outs() << Func.getName() << "\n";
        llvm::outs() << "----------------------------" << "\n";
        for(BasicBlock& BB : Func){
            llvm::outs() << "----------------------" << "\n";
            for(Instruction& inst : BB){
                llvm::outs() << inst << "\n";
            }
            llvm::outs() << "----------------------" << "\n";
        }
        llvm::outs() << "-----------------------------" << "\n";
    }
    llvm::outs() << "---------------------------------------------" << "\n";
}


template<class F,class B>
pair<F,B> Analysis<F,B>::CallInflowFunction(int context_label,Function* target_function,BasicBlock* bb,F a1,B d1)
{
    llvm::outs()<<"\nThis function CallInflowFunction() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F,class B>
pair<F,B> Analysis<F,B>::CallOutflowFunction(int context_label,Function* target_function,BasicBlock* bb,F a3,B d3,F a1,B d1)
{
    llvm::outs()<<"\nThis function CallOutflowFunction() has not been implemented. EXITING !!\n";
    exit(-1);
}

//=====================setter and getters for IN-OUT Maps==================================
template<class F,class B>
F Analysis<F,B>::getForwardComponentAtInOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return IN[label][&I].first;
//    return IN[make_pair(label,&I)].first;
}

template<class F,class B>
F Analysis<F,B>::getForwardComponentAtOutOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return OUT[label][&I].first;
//    return OUT[make_pair(label,&I)].first;
}

template<class F,class B>
B Analysis<F,B>::getBackwardComponentAtInOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return IN[label][&I].second;
//    return IN[make_pair(label,&I)].second;
}

template<class F,class B>
B Analysis<F,B>::getBackwardComponentAtOutOfThisInstruction(Instruction &I)
{
    int label=getProcessingContextLabel();
    return OUT[label][&I].second;
//    return OUT[make_pair(label,&I)].second;
}

template<class F,class B>
void Analysis<F,B>::setForwardComponentAtInOfThisInstruction(Instruction *I,F in_value)
{
    int label=getProcessingContextLabel();
    IN[label][I].first = in_value;
//    IN[make_pair(label,I)].first=in_value;
}

template<class F,class B>
void Analysis<F,B>::setForwardComponentAtOutOfThisInstruction(Instruction *I,F out_value)
{
    int label=getProcessingContextLabel();
    OUT[label][I].first = out_value;
//    OUT[make_pair(label,I)].first=out_value;
}

template<class F,class B>
void Analysis<F,B>::setBackwardComponentAtInOfThisInstruction(Instruction *I,B in_value)
{
    int label=getProcessingContextLabel();
    IN[label][I].second = in_value;
//    IN[make_pair(label,I)].second=in_value;
}

template<class F,class B>
void Analysis<F,B>::setBackwardComponentAtOutOfThisInstruction(Instruction *I,B out_value)
{
    int label=getProcessingContextLabel();
    OUT[label][I].second = out_value;
//    OUT[make_pair(label,I)].second=out_value;
}

//=====================setter and getters CS_BB==================================

template<class F,class B>
pair<F,B> Analysis<F,B>::getIn(int label, llvm::BasicBlock *BB) {
//    return IN[{label,&(*BB->begin())}];
    return CS_BB_IN[label][BB];
}

template<class F,class B>
pair<F,B> Analysis<F,B>::getOut(int label, llvm::BasicBlock *BB) {
    return CS_BB_OUT[label][BB];
}

template<class F, class B>
void Analysis<F,B>::setForwardIn(int label, llvm::BasicBlock *BB, F dataflowvalue) {
    CS_BB_IN[label][BB].first = dataflowvalue;
}

template<class F, class B>
void Analysis<F,B>::setForwardOut(int label, llvm::BasicBlock *BB, F dataflowvalue) {
    CS_BB_OUT[label][BB].first = dataflowvalue;
}

template<class F, class B>
void Analysis<F,B>::setBackwardIn(int label, llvm::BasicBlock * BB, B dataflowvalue) {
    CS_BB_IN[label][BB].second = dataflowvalue;
}

template<class F, class B>
void Analysis<F,B>::setBackwardOut(int label, llvm::BasicBlock * BB, B dataflowvalue) {
    CS_BB_OUT[label][BB].second = dataflowvalue;
}

//=================================================================================




//=====================setter and getters for context objects==================================
template<class F,class B>
F Analysis<F,B>::getForwardInflowForThisContext(int context_label)
{
//    return context_label_to_context_object_map[context_label].second.first.first;
    return context_label_to_context_object_map[context_label]->getInflowValue().first;
}

template<class F,class B>
B Analysis<F,B>::getBackwardInflowForThisContext(int context_label)
{
//    return context_label_to_context_object_map[context_label].second.first.second;
    return context_label_to_context_object_map[context_label]->getInflowValue().second;
}

template<class F,class B>
F Analysis<F,B>::getForwardOutflowForThisContext(int context_label)
{
//    return context_label_to_context_object_map[context_label].second.second.first;
    return context_label_to_context_object_map[context_label]->getOutflowValue().first;
}

template<class F,class B>
B Analysis<F,B>::getBackwardOutflowForThisContext(int context_label)
{
//    return context_label_to_context_object_map[context_label].second.second.second;
    return context_label_to_context_object_map[context_label]->getOutflowValue().second;
}


template<class F,class B>
void Analysis<F,B>::setForwardInflowForThisContext(int context_label,F forward_inflow)
{
//    context_label_to_context_object_map[context_label].second.first.first=forward_inflow;
    context_label_to_context_object_map[context_label]->setForwardInflow(forward_inflow);
}

template<class F,class B>
void Analysis<F,B>::setBackwardInflowForThisContext(int context_label,B backward_inflow)
{
//    context_label_to_context_object_map[context_label].second.first.second=backward_inflow;
    context_label_to_context_object_map[context_label]->setBackwardInflow(backward_inflow);
}

template<class F,class B>
void Analysis<F,B>::setForwardOutflowForThisContext(int context_label,F forward_outflow)
{
//    context_label_to_context_object_map[context_label].second.second.first=forward_outflow;
    context_label_to_context_object_map[context_label]->setForwardOutflow(forward_outflow);
}

template<class F,class B>
void Analysis<F,B>::setBackwardOutflowForThisContext(int context_label,B backward_outflow)
{
//    context_label_to_context_object_map[context_label].second.second.second=backward_outflow;
    context_label_to_context_object_map[context_label]->setBackwardOutflow(backward_outflow);
}

template<class F,class B>
Function* Analysis<F,B>::getFunctionAssociatedWithThisContext(int context_label)
{
    return context_label_to_context_object_map[context_label]->getFunction();
}

template<class F,class B>
void Analysis<F,B>::setFunctionAssociatedWithThisContext(int context_label,Function *f)
{
//    context_label_to_context_object_map[context_label].first=f;
}

template<class F,class B>
int Analysis<F,B>::getNumberOfContexts() {
    return ProcedureContext.size();
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
        if(function.size()>0){
            performSplittingBB(function);
        }
	}

    int i=0;
    for(Function &function: M)
	{
		if(function.getName()=="main")
		{
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
            INIT_CONTEXT(fptr,{forward_inflow_bi,backward_inflow_bi},{forward_outflow_bi,backward_outflow_bi});
//            INIT_CONTEXT(make_pair(fptr,make_pair(make_pair(forward_inflow_bi,backward_inflow_bi),make_pair(forward_outflow_bi,backward_outflow_bi))));
		}
    }
    if(std::is_same<F, NoAnalysisType>::value)
    {
        //backward analysis
        direction = "backward";
        setCurrentAnalysisDirection(2);
        int backward_iteration_count=0;
        int iteration = 1;
        while (backward_worklist.size()>0)
        {
//            doAnalysisBackward();
            backward_iteration_count++;
        }
    }
    else if(std::is_same<B, NoAnalysisType>::value)
    {
        //forward analysis
        direction = "forward";
        setCurrentAnalysisDirection(1);
        int forward_iteration_count=0;
        int iteration = 1;
        while (forward_worklist.size()>0)
        {
            doAnalysisForward();
            forward_iteration_count++;
        }
    }
    else
    {
        direction = "bidirectional";
//        int fi=1,bi=1;
//        int iteration = 1;
//        while (forward_worklist.size()>0||backward_worklist.size()>0)
//        {
//            llvm::outs() << REDB << "\n--------------------------------------Start Iteration-" << iteration << "--------------------------------------\n";
//            llvm::outs()<<MAGENTAB<<"\nBackward Iteration:"<<bi++<<RST;
//            // current_analysis_direction=2;
//            setCurrentAnalysisDirection(2);
//            while(backward_worklist.size()>0)
//            {
//                doAnalysisBackward();
//            }
//            llvm::outs()<<MAGENTAB<<"\nForward Iteration:"<<fi++<<RST;
//            // current_analysis_direction=1;
//            setCurrentAnalysisDirection(1);
//            while(forward_worklist.size()>0)
//            {
//                doAnalysisForward();
//            }
//            printContext();
//            llvm::outs() << REDB << "\n--------------------------------------End Iteration-" << iteration << "--------------------------------------\n";
//            iteration++;
//        }
    }
    printResults();
}




template<class F,class B>
void Analysis<F,B>::INIT_CONTEXT(llvm::Function *function, std::pair<F,B> Inflow, std::pair<F,B> Outflow) {
    context_label_counter++;
//    Context<F,B> context_object(context_label_counter,function,Inflow,Outflow);
    Context<F,B> *context_object = new Context<F,B>(context_label_counter,function,Inflow,Outflow);
    int current_context_label = context_object->getLabel();
    setProcessingContextLabel(current_context_label);
    if(debug){
        llvm::outs() << "INITIALIZING CONTEXT:-" << "\n";
        llvm::outs() << "LABEL: " << context_object->getLabel() << "\n";
        llvm::outs() << "FUNCTION: " << function->getName() << "\n";
        llvm::outs() << "Inflow Value: ";
        printDataFlowValuesForward(Inflow.first);
    }
    if(std::is_same<B, NoAnalysisType>::value){
        //forward analysis
        context_label_to_context_object_map[current_context_label] = context_object;

        setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
        ProcedureContext.insert(current_context_label);

        for(BasicBlock *BB:post_order(&context_object->getFunction()->getEntryBlock()))
        {
            BasicBlock &b=*BB;
            forward_worklist.push(make_pair(current_context_label,&b));
            set_forward_worklist_contains_this_entry(current_context_label,&b);
            if(direction == "bidirectional"){
                backward_worklist.push(make_pair(current_context_label,&b));
                set_backward_worklist_contains_this_entry(current_context_label,&b);
            }
            setForwardIn(current_context_label,&b,getInitialisationValueForward());
            setForwardOut(current_context_label,&b,getInitialisationValueForward());
//            CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//            CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();

            //initialise IN-OUT maps for every instruction
            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
            {
                setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
            }
        }
        if(current_context_label==0)//main function with first invocation
        {
            setForwardInflowForThisContext(current_context_label,getBoundaryInformationForward());//setting inflow forward
        }
        else
        {
//            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);//setting inflow forward
            setForwardInflowForThisContext(current_context_label,context_object->getInflowValue().first);
        }
        setForwardIn(current_context_label, &context_object->getFunction()->getEntryBlock(), getForwardInflowForThisContext(current_context_label));
//        CS_BB_IN[make_pair(current_context_label,&context_object.getFunction()->getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
    } else if(std::is_same<F, NoAnalysisType>::value) {
        //backward analysis
//        context_label_to_context_object_map[current_context_label].first=&function;
//        context_label_to_context_object_map[current_context_label] = context_object;
//        setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
//        context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
//        ProcedureContext.insert(current_context_label);
//
//        for(BasicBlock *BB:inverse_post_order(&context_object.getFunction().back()))
//        {
//            BasicBlock &b=*BB;
//            forward_worklist.push(make_pair(current_context_label,&b));
//            backward_worklist.push(make_pair(current_context_label,&b));
//            forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//            backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//            CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//            CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//
//            //initialise IN-OUT maps for every instruction
//            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//            {
//                setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
//            }
//        }
//        if(current_context_label==0)//main function with first invocation
//        {
//            setBackwardInflowForThisContext(current_context_label,getBoundaryInformationBackward());//setting inflow backward
//        }
//        else
//        {
//            setBackwardInflowForThisContext(current_context_label,context_object.second.first.second);//setting inflow backward
//        }
//        CS_BB_OUT[make_pair(current_context_label,&context_object.getFunction().back())].second=getBackwardInflowForThisContext(current_context_label);
    } else{
        // Todo for Bidirectional Analysis
    }

}
//pair<Function*,pair<pair<F,B>,pair<F,B>>> context_object
//template <class F,class B>
//void Analysis<F,B>::INIT_CONTEXT(Context<F,B> context_object)//,F forward_component,B backward_component)
//{
//    Function &function=*(context_object.getFunction());
////    context_label_counter++;
//    int current_context_label = context_object.getLabel();
//    setProcessingContextLabel(current_context_label);
//
//    if(std::is_same<F, NoAnalysisType>::value)
//    {
//        //backward analysis
//
//        context_label_to_context_object_map[current_context_label].first=&function;
//        setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
//        context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
//        ProcedureContext.insert(current_context_label);
//
//        for(BasicBlock *BB:inverse_post_order(&function.back()))
//        {
//            BasicBlock &b=*BB;
//            forward_worklist.push(make_pair(current_context_label,&b));
//            backward_worklist.push(make_pair(current_context_label,&b));
//            forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//            backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//            CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//            CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//
//            //initialise IN-OUT maps for every instruction
//            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//            {
//                setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
//            }
//        }
//        if(current_context_label==0)//main function with first invocation
//        {
//            setBackwardInflowForThisContext(current_context_label,getBoundaryInformationBackward());//setting inflow backward
//        }
//        else
//        {
//            setBackwardInflowForThisContext(current_context_label,context_object.second.first.second);//setting inflow backward
//            // context_label_to_context_object_map[current_context_label].second.first.second=context_object.second.first.second;//setting inflow backward
//        }
//        // llvm::outs()<<MAGENTAB<<"\nSetting CS BB OUT Backward:"<<context_object.second.first.second[0]<<" for label:"<<current_context_label;
//        CS_BB_OUT[make_pair(current_context_label,&function.back())].second=getBackwardInflowForThisContext(current_context_label);
//        // CS_BB_OUT[make_pair(current_context_label,&function.back())].second=context_label_to_context_object_map[current_context_label].second.first.second;
//
//    }
//    else if(std::is_same<B, NoAnalysisType>::value)
//    {
//        //forward analysis
////        context_label_to_context_object_map[current_context_label].first=&function;
//        context_label_to_context_object_map[current_context_label] = context_object;
//
//        setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
//        ProcedureContext.insert(current_context_label);
//
//        for(BasicBlock *BB:post_order(&function.getEntryBlock()))
//        {
//            BasicBlock &b=*BB;
//            forward_worklist.push(make_pair(current_context_label,&b));
//            backward_worklist.push(make_pair(current_context_label,&b));
//            forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//            backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//            CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//            CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//
//            //initialise IN-OUT maps for every instruction
//            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//            {
//                setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
//                setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
//            }
//        }
//        if(current_context_label==1)//main function with first invocation
//        {
//            setForwardInflowForThisContext(current_context_label,getBoundaryInformationForward());//setting inflow forward
//        }
//        else
//        {
////            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);//setting inflow forward
//            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);
//        }
//        CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
//    }
//    else
//    {
//        //bidirectional analysis
//        context_label_to_context_object_map[current_context_label].first=&function;
//
//        if(getCurrentAnalysisDirection()==10)
//        {
////            llvm::outs()<<CYANB<<"\nforward init-context"<<RST;
//            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
//            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
//            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
//            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
//
//            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
//            ProcedureContext.insert(current_context_label);
//            for(BasicBlock *BB:post_order(&function.getEntryBlock()))
//            {
//                BasicBlock &b=*BB;
//                // llvm::outs()<<"\nBasicBlock: ";
//                // b.printAsOperand(llvm::outs(),false);
//                forward_worklist.push(make_pair(current_context_label,&b));
//                // backward_worklist.push(make_pair(current_context_label,&b));
//                forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                // backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//
//                //initialise IN-OUT maps for every instruction
//                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//                {
//                    // llvm::outs()<<"\n"<<*inst;
//                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    // IN[&(*inst)].first=getInitialisationValueForward();
//                    // OUT[&(*inst)].first=getInitialisationValueForward();
//                    // IN[&(*inst)].second=getInitialisationValueBackward();
//                    // OUT[&(*inst)].second=getInitialisationValueBackward();
//                }
//            }
//        }
//        else if(getCurrentAnalysisDirection()==20)
//        {
////            llvm::outs()<<CYANB<<"\nbackward init-context"<<RST;
//            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
//            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
//            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
//            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
//
//            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
//            ProcedureContext.insert(current_context_label);
//            for(BasicBlock *BB:inverse_post_order(&function.back()))
//            {
//                BasicBlock &b=*BB;
//                // llvm::outs()<<"\nBasicBlock: ";
//                // b.printAsOperand(llvm::outs(),false);
//                // forward_worklist.push(make_pair(current_context_label,&b));
//                backward_worklist.push(make_pair(current_context_label,&b));
//                // forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//
//                //initialise IN-OUT maps for every instruction
//                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//                {
//                    // llvm::outs()<<"\n"<<*inst;
//                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    // IN[&(*inst)].first=getInitialisationValueForward();
//                    // OUT[&(*inst)].first=getInitialisationValueForward();
//                    // IN[&(*inst)].second=getInitialisationValueBackward();
//                    // OUT[&(*inst)].second=getInitialisationValueBackward();
//                }
//            }
//
//        }
//        else
//        {
//            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
//            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
//            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
//            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
//
//
//
//
//            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
//            ProcedureContext.insert(current_context_label);
//            for(BasicBlock *BB:inverse_post_order(&function.back()))
//            {
//                //populate backward worklist
//                BasicBlock &b=*BB;
//                // llvm::outs()<<"\nBasicBlock: ";
//                // b.printAsOperand(llvm::outs(),false);
//                // forward_worklist.push(make_pair(current_context_label,&b));
//                backward_worklist.push(make_pair(current_context_label,&b));
//                // forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                // CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                // CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//
//                //initialise IN-OUT maps for every instruction
//                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//                {
//                    // llvm::outs()<<"\n"<<*inst;
//                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
//                    // setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    // setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    // IN[&(*inst)].first=getInitialisationValueForward();
//                    // OUT[&(*inst)].first=getInitialisationValueForward();
//                    // IN[&(*inst)].second=getInitialisationValueBackward();
//                    // OUT[&(*inst)].second=getInitialisationValueBackward();
//                }
//            }
//            for(BasicBlock *BB:post_order(&function.getEntryBlock()))
//            {
//                //populate forward worklist
//                BasicBlock &b=*BB;
//                // llvm::outs()<<"\nBasicBlock: ";
//                // b.printAsOperand(llvm::outs(),false);
//                forward_worklist.push(make_pair(current_context_label,&b));
//                // backward_worklist.push(make_pair(current_context_label,&b));
//                forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                // backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                // CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                // CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//
//                //initialise IN-OUT maps for every instruction
//                for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//                {
//                    // llvm::outs()<<"\n"<<*inst;
//                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
//                    // IN[&(*inst)].first=getInitialisationValueForward();
//                    // OUT[&(*inst)].first=getInitialisationValueForward();
//
//                    // IN[&(*inst)].second=getInitialisationValueBackward();
//                    // OUT[&(*inst)].second=getInitialisationValueBackward();
//                }
//            }
//        }
//        //irresepective of the current direction of analysis, the INFLOW values need to be set.
//        if(current_context_label==0)//main function with first invocation
//        {
//            setForwardInflowForThisContext(current_context_label,getBoundaryInformationForward());//setting inflow forward
//            setBackwardInflowForThisContext(current_context_label,getBoundaryInformationBackward());//setting inflow backward
//            // context_label_to_context_object_map[current_context_label].second.first.second=getBoundaryInformationBackward();//setting inflow backward
//            // context_label_to_context_object_map[current_context_label].second.first.first=getBoundaryInformationForward();//setting inflow forward
//        }
//        else
//        {
//            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);//setting inflow forward
//            setBackwardInflowForThisContext(current_context_label,context_object.second.first.second);//setting inflow backward
//            // context_label_to_context_object_map[current_context_label].second.first.first=context_object.second.first.first;//setting inflow forward
//            // context_label_to_context_object_map[current_context_label].second.first.second=context_object.second.first.second;//setting inflow backward
//        }
//        CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
//        CS_BB_OUT[make_pair(current_context_label,&function.back())].second=getBackwardInflowForThisContext(current_context_label);
//        // CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=context_label_to_context_object_map[current_context_label].second.first.first;
//        // CS_BB_OUT[make_pair(current_context_label,&function.back())].second=context_label_to_context_object_map[current_context_label].second.first.second;
//
//    }
//}

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
        delete_forward_worklist_contains_this_entry(current_context_label,bb);

        BasicBlock &b=*bb;
        Function *f=context_label_to_context_object_map[current_context_label]->getFunction();
        Function &function=*f;
        
        //step 5
        if(bb!=(&function.getEntryBlock()))
        {
            //step 6
            setForwardIn(current_pair.first,current_pair.second,getInitialisationValueForward());
//            CS_BB_IN[current_pair].first=getInitialisationValueForward();
            
            //step 7 and 8
            for(auto pred_bb:predecessors(bb))
            {
                setForwardIn(current_pair.first,current_pair.second, performMeetForward(getIn(current_pair.first,current_pair.second).first,getOut(current_pair.first,pred_bb).first)); // CS_BB_OUT[make_pair(current_pair.first,pred_bb)].first
//                CS_BB_IN[current_pair].first=performMeetForward(CS_BB_IN[current_pair].first,CS_BB_OUT[make_pair(current_pair.first,pred_bb)].first);
            }
        }
        else
        {
            //In value of this node is same as INFLOW value
            setForwardIn(current_pair.first,current_pair.second,getForwardInflowForThisContext(current_context_label));
//            CS_BB_IN[current_pair].first=getForwardInflowForThisContext(current_context_label);
            
        }
        
        //step 9
        F a1 = getIn(current_pair.first,current_pair.second).first; //CS_BB_IN[current_pair].first;
        if(debug){
            printLine(current_pair.first);
            llvm::outs() << "TESTING\n";
            llvm::outs() << "INFLOW VALUES: ";
            printDataFlowValuesForward(a1);
            printLine(current_pair.first);
        }
        B d1 = getOut(current_pair.first,current_pair.second).second; //CS_BB_OUT[current_pair].second;

        F previous_value_at_out_of_this_node = getOut(current_pair.first,current_pair.second).first; //CS_BB_OUT[current_pair].first;

        //step 10
        bool contains_a_method_call = false;
        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
        {
            Instruction &I=*inst;
            if ( CallInst *ci = dyn_cast<CallInst>(&I))
            {
                Function* target_function= ci->getCalledFunction();
                if(not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(&I))
                {
                    continue; //this is an inbuilt function so doesn't need to be processed.
                }
                contains_a_method_call=true;
                break;
            }
        }
        if(contains_a_method_call)
        {
            F prev = getForwardComponentAtInOfThisInstruction(*(b.begin()));
//            F prev = getIn(current_pair.first,current_pair.second).first; //CS_BB_IN[current_pair].first; //a1
            //step 11
            for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
            {
                Instruction &I=*inst;
                if ( CallInst *ci = dyn_cast<CallInst>(&I))
                {
                    Function* target_function= ci->getCalledFunction();
                    if(not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(&I))
                    {
                        continue; //this is an inbuilt function so doesn't need to be processed.
                    }
                    
                    /*
                    At the call instruction, the value at IN should be splitted into two components:
                    1) Purely Global and 2) Mixed.
                    The purely global component is given to the start of callee.
                    */
                    
                    //step 12
                    pair<F,B> inflow_pair = CallInflowFunction(current_context_label,target_function,bb,a1,d1);
                    F a2=inflow_pair.first;
                    B d2=inflow_pair.second;

                    F new_outflow_forward;
                    B new_outflow_backward;
                    
                    //step 13
//                    pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(a2,d2),make_pair(new_outflow_forward,new_outflow_backward)));

                    setForwardComponentAtInOfThisInstruction(&(*inst),prev);//compute IN from previous OUT-value
                    
                    int matching_context_label=0;
//                    matching_context_label = check_if_context_already_exists(new_context_object);
                    matching_context_label = check_if_context_already_exists(target_function,{a2,d2},{new_outflow_forward,new_outflow_backward});
                    if(matching_context_label>0)//step 15
                    {
                        if(debug){
                            printLine(current_context_label);
                            llvm::outs() << *inst << "\n";
                            llvm::outs() << "IN: ";
                        }
                        //step 14
                        pair<int,Instruction*>mypair = make_pair(current_context_label,&(*inst));
                        context_transition_graph[mypair] = matching_context_label;
                        if(debug){
                            printDataFlowValuesForward(a1);
                        }

                        //step 16 and 17
                        F a3=getForwardOutflowForThisContext(matching_context_label);
                        B d3=getBackwardOutflowForThisContext(matching_context_label);

                        pair<F,B> outflow_pair=CallOutflowFunction(current_context_label,target_function,bb,a3,d3,a1,d1);
                        F value_to_be_meet_with_prev_out=outflow_pair.first;
                        B d4=outflow_pair.second;

                        //step 18 and 19
                        
                        /*
                        At the call instruction, the value at IN should be splitted into two components.
                        The purely global component is given to the callee while the mixed component is propagated
                        to OUT of this instruction after executing computeOUTfromIN() on it.
                        */

                        F a5 = getPurelyLocalComponentForward(a1);
                        
                        /*
                        At the OUT of this instruction, the value from END of callee procedure is to be merged
                        with the local(mixed) value propagated from IN. Note that this merging "isn't" 
                        exactly (necessarily) the meet between these two values.
                        */


                        /*
                        As explained in ip-vasco,pdf, we need to perform meet with the original value of OUT
                        of this instruction to avoid the oscillation problem.
                        */
                        setForwardComponentAtOutOfThisInstruction(&(*inst),performMeetForward(performMeetForward(value_to_be_meet_with_prev_out,getForwardComponentAtOutOfThisInstruction(*inst)),a5));

                        prev=getForwardComponentAtOutOfThisInstruction((*inst));
                        if(debug){
                            llvm::outs() << "OUT: ";
                            printDataFlowValuesForward(prev);
                            printLine(current_context_label);
                        }
//                        if(matching_context_label != current_context_label){
//                            freeMemory(matching_context_label);
//                        }
                    }
                    else//step 20
                    {
                        //creating a new context
//                        INIT_CONTEXT(new_context_object); //step 21
                        INIT_CONTEXT(target_function,{a2,d2},{new_outflow_forward,new_outflow_backward}); //step 21

                        //step 14
                        //This step must be done after above step because context label counter gets updated after INIT-Context is invoked.
                        pair<int,Instruction*>mypair = make_pair(current_context_label,&(*inst));
                        context_transition_graph[mypair] = getContextLabelCounter();
                        return;
                    }
                }
                else
                {
                    if(debug){
                        printLine(current_context_label);
                        llvm::outs() << *inst << "\n";
                        llvm::outs() << "IN: ";
                        printDataFlowValuesForward(prev);
                    }
                    setForwardComponentAtInOfThisInstruction(&(*inst),prev);//compute IN from previous OUT-value
                    F new_prev = computeOutFromIn(*inst);
                    setForwardComponentAtOutOfThisInstruction(&(*inst),new_prev);
                    if(debug){
                        llvm::outs() << "OUT: ";
                        printDataFlowValuesForward(new_prev);
                    }
                    prev=getForwardComponentAtOutOfThisInstruction(*inst);
                    if(debug){
                        printLine(current_context_label);
                    }
                }
            }
            setForwardOut(current_pair.first,current_pair.second,prev);
//            CS_BB_OUT[current_pair].first=prev;
        }
        else//step 22
        {
            //step 23
            //NormalFlowFunction
            setForwardOut(current_pair.first,current_pair.second,NormalFlowFunctionForward(current_pair));
//            CS_BB_OUT[current_pair].first=NormalFlowFunctionForward(current_pair);
        }
        bool changed=false;
        if(!EqualDataFlowValuesForward(previous_value_at_out_of_this_node,getOut(current_pair.first,current_pair.second).first)) // CS_BB_OUT[current_pair].first)
        {
            changed=true;
        }
        if(changed)//step 24
        {
            //not yet converged
            for(auto succ_bb:successors(bb))//step 25
            {
                //step 26
                if(!forward_worklist_contains_this_entry(current_context_label,succ_bb))
                {
                    forward_worklist.push(make_pair(current_context_label,succ_bb));
                    set_forward_worklist_contains_this_entry(current_context_label,succ_bb);
                }
                if(direction == "bidirectional"){
                    if(!backward_worklist_contains_this_entry(current_context_label,succ_bb))
                    {
                        backward_worklist.push(make_pair(current_context_label,succ_bb));
                        set_backward_worklist_contains_this_entry(current_context_label,succ_bb);
                    }
                }
            }

        }
        if(bb==&function.back())//step 27
        {
            //last node/exit node
            
            //step 28
            setForwardOutflowForThisContext(current_context_label,getPurelyGlobalComponentForward(getOut(current_pair.first,current_pair.second).first));//setting forward outflow CS_BB_OUT[current_pair].first
//            context_label_to_context_object_map[current_context_label].second.second.first=getPurelyGlobalComponentForward(CS_BB_OUT[current_pair].first);//setting forward outflow
            bool flag = false;
            for(auto context_inst_pairs:context_transition_graph)//step 29
            {
                if(context_inst_pairs.second == current_context_label)//matching the called function
                {
                    //step 30
                    BasicBlock *bb=context_inst_pairs.first.second->getParent();
                    pair<int,BasicBlock*>context_bb_pair=make_pair(context_inst_pairs.first.first,bb);
                    if(!forward_worklist_contains_this_entry(context_bb_pair.first,context_bb_pair.second))
                    {
                        forward_worklist.push(context_bb_pair);
                        set_forward_worklist_contains_this_entry(context_bb_pair.first,context_bb_pair.second);
                    }
                    if(direction == "bidirectional"){
                        if(!backward_worklist_contains_this_entry(context_bb_pair.first,context_bb_pair.second))
                        {
                            backward_worklist.push(context_bb_pair);
                            set_backward_worklist_contains_this_entry(context_bb_pair.first,context_bb_pair.second);
                        }
                    }
                }
                if(context_inst_pairs.first.first == current_context_label){
                    flag = true;
                }
            }
            if(not flag){
//                freeMemory(current_context_label);
            }
        }
    }
}

template <class F,class B>
F Analysis<F,B>::NormalFlowFunctionForward(pair<int,BasicBlock*> current_pair_of_context_label_and_bb)
{
    BasicBlock &b=*(current_pair_of_context_label_and_bb.second);
    F prev = getIn(current_pair_of_context_label_and_bb.first,current_pair_of_context_label_and_bb.second).first; // CS_BB_IN[current_pair_of_context_label_and_bb].first;
    //traverse a basic block in forward direction
    for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
    {
        if(debug){
            printLine(current_pair_of_context_label_and_bb.first);
            llvm::outs() << *inst << "\n";
            llvm::outs() << "IN: ";
            printDataFlowValuesForward(prev);
        }
        setForwardComponentAtInOfThisInstruction(&(*inst),prev);//compute IN from previous OUT-value
        F new_prev = computeOutFromIn(*inst);
        setForwardComponentAtOutOfThisInstruction(&(*inst),new_prev);
        if(debug){
            llvm::outs() << "OUT: ";
            printDataFlowValuesForward(new_prev);
        }
        prev=getForwardComponentAtOutOfThisInstruction(*inst);
        if(debug){
            printLine(current_pair_of_context_label_and_bb.first);
        }
    }
    return prev;
}


template<class F,class B>
int Analysis<F,B>::check_if_context_already_exists(llvm::Function *function, pair<F,B> Inflow, pair<F,B> Outflow){
    if(std::is_same<B, NoAnalysisType>::value)
    {
        //forward only
        for(auto set_itr:ProcedureContext)
        {
            Context<F,B> *current_object = context_label_to_context_object_map[set_itr];
            F new_context_values = Inflow.first;
            F current_context_values = current_object->getInflowValue().first;
            if(function->getName() == current_object->getFunction()->getName() && EqualDataFlowValuesForward(new_context_values,current_context_values)){
                if(debug){
                    llvm::outs() << "======================================================================================" << "\n";
                    llvm::outs() << "Context found:-" << "\n";
                    llvm::outs() << "LABEL: " << set_itr << "\n";
                    llvm::outs() << "======================================================================================" << "\n";
                }
                return set_itr;
            }
        }
    } else if(std::is_same<F, NoAnalysisType>::value){
        // Todo for Backward Analysis
    } else{
        // Todo for Bidirectional Analysis
    }
    if(debug){
        llvm::outs() << "======================================================================================" << "\n";
        llvm::outs() << "Context Not found:-" << "\n";
        llvm::outs() << "FUNCTION: " << function->getName() << "\n";
        llvm::outs() << "INFLOW VALUE: ";
        printDataFlowValuesForward(Inflow.first);
        llvm::outs() << "======================================================================================" << "\n";
    }
}
//template <class F,class B>
//int Analysis<F,B>::check_if_context_already_exists(pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object)
//{
//    if(std::is_same<B, NoAnalysisType>::value)
//    {
//        //forward only
//        for(auto set_itr:ProcedureContext)
//        {
//            pair<Function*,pair<pair<F,B>,pair<F,B>>> current_object= context_label_to_context_object_map[set_itr];
//            F new_context_values=new_context_object.second.first.first;
//            F current_context_values=current_object.second.first.first;
//            if(new_context_object.first==current_object.first && EqualDataFlowValuesForward(new_context_values,current_context_values))
//            {
//                return set_itr;
//            }
//        }
//    }
//    else if(std::is_same<F, NoAnalysisType>::value)
//    {
//        //backward only
//        for(auto set_itr:ProcedureContext)
//        {
//            pair<Function*,pair<pair<F,B>,pair<F,B>>> current_object= context_label_to_context_object_map[set_itr];
//            B new_context_values= new_context_object.second.first.second;
//            B current_context_values= current_object.second.first.second;
//            if(new_context_object.first==current_object.first&&EqualDataFlowValuesBackward(new_context_values,current_context_values))
//            {
//                return set_itr;
//            }
//        }
//    }
//    else
//    {
//        //bidirectional
//        int i=0;
//        for(auto set_itr:ProcedureContext)
//        {
//            pair<Function*,pair<pair<F,B>,pair<F,B>>> current_object= context_label_to_context_object_map[set_itr];
//            F new_context_values_forward = new_context_object.second.first.first;
//            F current_context_values_forward = current_object.second.first.first;
//            B new_context_values_backward = new_context_object.second.first.second;
//            B current_context_values_backward = current_object.second.first.second;
//            if(new_context_object.first==current_object.first&&EqualDataFlowValuesBackward(new_context_values_backward,current_context_values_backward)&&EqualDataFlowValuesForward(new_context_values_forward,current_context_values_forward))
//            {
//                return set_itr;
//            }
//        }
//    }
//    return 0;
//}

template <class F,class B>
void Analysis<F,B>::printWorklistMaps()
{

}

//template <class F,class B>
//void Analysis<F,B>::doAnalysisBackward()
//{
//    while(!backward_worklist.empty())//step 2
//    {
//        //step 3 and 4
//        pair<int,BasicBlock*> current_pair=backward_worklist.top();
//        int current_context_label;
//        BasicBlock *bb;
//        current_context_label=backward_worklist.top().first;
//        setProcessingContextLabel(current_context_label);
//        bb=backward_worklist.top().second;
//        backward_worklist.pop();
//        backward_worklist_contains_this_entry[make_pair(current_context_label,bb)]=false;
//
//        BasicBlock &b=*bb;
//        Function *f=context_label_to_context_object_map[current_context_label].first;
//        Function &function=*f;
//        llvm::outs()<<BLUEB"\nContext Label: "<<current_context_label<<" BasicBlock: ";
//        b.printAsOperand(llvm::outs(),false);
//        llvm::outs()<<" Function Name: "<<f->getName()<<" WS:"<<backward_worklist.size()<<RST;
//
//        //step 5
//        if(bb!=(&function.back()))
//        {
//            llvm::outs()<<"\nNon-Exit Block";
//            //step 6
//            CS_BB_OUT[current_pair].second=getInitialisationValueBackward();
//            //step 7 and 8
//            for(auto succ_bb:successors(bb))
//            {
//                llvm::outs()<<"\nSucc BB: ";
//                succ_bb->printAsOperand(llvm::outs(),false);
//                llvm::outs() << "\t"; //Label of Basic block
//                CS_BB_OUT[current_pair].second=performMeetBackward(CS_BB_OUT[current_pair].second,CS_BB_IN[make_pair(current_pair.first,succ_bb)].second);
//            }
//        }
//        else
//        {
//            llvm::outs()<<"\nExit Block";
//            //Out value of this node is same as INFLOW value
//            CS_BB_OUT[current_pair].second=getBackwardInflowForThisContext(current_context_label);
//            // CS_BB_OUT[current_pair].second=context_label_to_context_object_map[current_context_label].second.first.second;
//        }
//
//        //step 9
//        F a1=CS_BB_IN[current_pair].first;
//        B d1=CS_BB_OUT[current_pair].second;
//
//        B previous_value_at_in_of_this_node=CS_BB_IN[current_pair].second;
//        //step 10
//        bool contains_a_method_call=false;
//        for(auto inst=&*(b.begin());inst!=NULL;inst=inst->getNextNonDebugInstruction())
//        {
//            Instruction &I=*inst;
//            if ( CallInst *ci = dyn_cast<CallInst>(&I))
//            {
//                Function* target_function= ci->getCalledFunction();
//                // llvm::outs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
//                // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
//                // {
//                //     continue;//this is an inbuilt function so doesnt need to be processed.
//                // }
//                if((*target_function).size()==0 || isAnIgnorableDebugInstruction(&I))
//                {
//                    continue;//this is an inbuilt function so doesnt need to be processed.
//                }
//                // contains_a_method_call=true;
//                contains_a_method_call=true;
//                llvm::outs()<<REDB"\nMethod Call found in basic block: "<<inst->getParent()->getName()<<RST;
//                break;
//            }
//        }
//        // llvm::outs()<<MAGENTAB"\nContains a method call: "<<contains_a_method_call<<RST;
//        if(contains_a_method_call)
//        {
//            B prev=CS_BB_OUT[current_pair].second;
//
//            //step 11
//            for(auto inst=b.rbegin();inst!=b.rend();inst++)
//            {
//                Instruction &I=*inst;
//                if ( CallInst *ci = dyn_cast<CallInst>(&I))
//                {
//
//                    Function* target_function= ci->getCalledFunction();
//                    // llvm::outs()<<REDB"\nCalled Function: "<<target_function->getName()<<RST;
//                    // if((*target_function).size()==0||isAnIgnorableDebugInstruction(target_function->getName().str(),"llvm.dbg"))
//                    // {
//                    //     continue;
//                    // }
//                    if((*target_function).size()==0||isAnIgnorableDebugInstruction(&I))
//                    {
//                        continue;
//                    }
//
//                    llvm::outs()<<"\n"<<*inst;
//
//                    /*
//                    At the call instruction, the value at OUT should be splitted into two components:
//                    1) Purely Global and 2) Mixed.
//                    The purely global component is given to the end of callee.
//                    */
//                    //step 12
//                    pair<F,B> inflow_pair=CallInflowFunction(current_context_label,target_function,bb,a1,d1);
//                    F a2=inflow_pair.first;
//                    B d2=inflow_pair.second;
//
//                    // F new_inflow_forward;
//                    // B new_inflow_backward=getPurelyGlobalComponentBackward(prev);//prev;
//                    F new_outflow_forward;
//                    B new_outflow_backward;
//
//
//
//                    //step 13
//                    // pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(new_inflow_forward,new_inflow_backward),make_pair(new_outflow_forward,new_outflow_backward)));
//                    pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(a2,d2),make_pair(new_outflow_forward,new_outflow_backward)));
//
//                    //===========================================================
//                    // OUT[&(*inst)].second=prev;//compute OUT from previous IN-value
//                    setBackwardComponentAtOutOfThisInstruction(&(*inst),prev);//compute OUT from previous IN-value
//                    //===========================================================
//                    // llvm::outs()<<WHITEB<<"\nINFLOW:"<<prev[0];
//                    int matching_context_label=0;
//                    matching_context_label=check_if_context_already_exists(new_context_object);
//                    llvm::outs() << "MATCHING CONTEXT LABEL: ";
//                    llvm::outs() << matching_context_label << "\n";
//                    if(matching_context_label>0)//step 15
//                    {
//                        llvm::outs()<<BLUEB"\nExisting context found!"<<matching_context_label<<RST;
//                        //step 14
//                        pair<int,Instruction*>mypair=make_pair(current_context_label,&(*inst));
//                        context_transition_graph[mypair]=matching_context_label;
//
//                        //step 16 and 17
//                        F a3=getForwardOutflowForThisContext(matching_context_label);
//                        B d3=getBackwardOutflowForThisContext(matching_context_label);
//                        // F a3=context_label_to_context_object_map[matching_context_label].second.second.first;
//                        // B d3=context_label_to_context_object_map[matching_context_label].second.second.second;
//
//                        pair<F,B> outflow_pair=CallOutflowFunction(current_context_label,target_function,bb,a3,d3,a1,d1);
//                        F a4=outflow_pair.first;
//                        B value_to_be_meet_with_prev_in=outflow_pair.second;
//
//                        // new_outflow_backward=getBackwardOutflowForThisContext(matching_context_label);
//                        // llvm::outs()<<WHITEB<<"\nNew Outflow before meet:";
//                        // // <<new_outflow_backward[0]<<RST;
//                        // for(auto x:value_to_be_meet_with_prev_in)
//                        // {
//                        //     llvm::outs()<<"\t"<<x->getName();
//                        // }
//                        // llvm::outs()<<RST;
//
//
//
//                        //===========================================================
//                        //step 18 and 19
//
//                        /*
//                        At the call instruction, the value at OUT should be splitted into two components.
//                        The purely global component is given to the callee while the mixed component is propagated
//                        to IN of this instruction after executing computeINfromOUT() on it.
//                        */
//
//
//                        // B value_to_be_propagated_to_in_of_instruction=computeInFromOut(*inst);//d5
//
//                        /*
//                        At the IN of this instruction, the value from START of callee procedure is to be merged
//                        with the local(mixed) value propagated from OUT. Note that this merging "isn't"
//                        exactly (necessarily) the meet between these two values.
//                        */
//
//                        // B merged_value_at_in_of_instruction=getCombinedValuesAtCallBackward(new_outflow_backward,value_to_be_propagated_to_in_of_instruction);//combining d4 and d5
//
//
//                        /*
//                        As explained in ip-vasco,pdf, we need to perform meet with the original value of IN
//                        of this instruction to avoid the oscillation problem.
//                        */
//
//                        setBackwardComponentAtInOfThisInstruction(&(*inst),performMeetBackward(value_to_be_meet_with_prev_in,getBackwardComponentAtInOfThisInstruction((*inst))));
//
//                        prev=getBackwardComponentAtInOfThisInstruction((*inst));
//                        // llvm::outs()<<WHITEB<<"\nNew Inflow after meet:"<<prev[0]<<RST;
//
//                        //===========================================================
//                    }
//                    else//step 20
//                    {
//                        llvm::outs() <<REDB"\n----------------------------------------------\n";
//                        llvm::outs() << MAGENTAB"CREATING A NEW CONTEXT!!!!!!!" << "\n";
//                        //creating a new context
//                        INIT_CONTEXT(new_context_object);//step 21
//
//                        pair<int,Instruction*>mypair=make_pair(current_context_label,&(*inst));
//                        //step 14
//                        context_transition_graph[mypair]=context_label_counter;
//                        llvm::outs() << "LABEL:- " << context_label_counter << "\n";
//                        llvm::outs() << "INSTRUCTION:- " << *inst << "\n";
//                        llvm::outs() <<REDB"\n----------------------------------------------\n";
//                        return;
//                        //===========================================================
//                        // IN[&(*inst)].second=computeInFromOut(*inst);
//                        // prev=IN[&(*inst)].second;
//                        //===========================================================
//                    }
//                }
//                else
//                {
//                    // llvm::outs() << CYANB <<"\n"<<*inst << "AYUSH";
//                    // OUT[&(*inst)].second=prev;//compute OUT from previous IN-value
//                    setBackwardComponentAtOutOfThisInstruction(&(*inst),prev);//compute OUT from previous IN-value
//                    // IN[&(*inst)].second=computeInFromOut(*inst);
//                    setBackwardComponentAtInOfThisInstruction(&(*inst),computeInFromOut(*inst));
//                    // prev=IN[&(*inst)].second;
//                    prev=getBackwardComponentAtInOfThisInstruction((*inst));
//                }
//            }
//            CS_BB_IN[current_pair].second=prev;
//        }
//        else//step 22
//        {
//            //step 23
//            //NormalFlowFunction
//            CS_BB_IN[current_pair].second=NormalFlowFunctionBackward(current_pair);
//
//        }
//        bool changed=false;
//        if(!EqualDataFlowValuesBackward(previous_value_at_in_of_this_node,CS_BB_IN[current_pair].second))
//        {
//            changed=true;
//        }
//        if(changed)//step 24
//        {
//            //not yet converged
//            for(auto pred_bb:predecessors(bb))//step 25
//            {
//                llvm::outs()<<"\nPushing Pred BB: ";
//                pred_bb->printAsOperand(llvm::outs(),false);
//                llvm::outs() << "\t"; //Label of Basic block
//
//                //step 26
//                if(!forward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)])
//                {
//                    forward_worklist.push(make_pair(current_context_label,pred_bb));
//                    forward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)]=true;
//                }
//                if(!backward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)])
//                {
//                    backward_worklist.push(make_pair(current_context_label,pred_bb));
//                    backward_worklist_contains_this_entry[make_pair(current_context_label,pred_bb)]=true;
//                }
//                // forward_worklist.push(make_pair(current_context_label,pred_bb));
//                // backward_worklist.push(make_pair(current_context_label,pred_bb));
//            }
//
//        }
//
//        if(bb==&function.getEntryBlock())//step 27
//        {
//            //first node/start node
//
//            //step 28
//            // llvm::outs()<<WHITEB<<"\nBACKWARD OUTFLOW VALUE(before):"<<context_label_to_context_object_map[current_context_label].second.second.second[0];
//            // llvm::outs()<<WHITEB<<"\nSetting outflow!";
//            // for(auto x:CS_BB_IN[current_pair].second)
//            // {
//            //     llvm::outs()<<"\t"<<x->getName();
//            // }
//            // llvm::outs()<<RST;
//            setBackwardOutflowForThisContext(current_context_label,getPurelyGlobalComponentBackward(CS_BB_IN[current_pair].second));//setting backward outflow
//            // context_label_to_context_object_map[current_context_label].second.second.second=getPurelyGlobalComponentBackward(CS_BB_IN[current_pair].second);//setting backward outflow
//
//            // llvm::outs()<<WHITEB<<"\nBACKWARD OUTFLOW VALUE(after):"<<context_label_to_context_object_map[current_context_label].second.second.second[0];
//
//            for(auto context_inst_pairs:context_transition_graph)//step 29
//            {
//                if(context_inst_pairs.second==current_context_label)//matching the called function
//                {
//                    //step 30
//
//                    llvm::outs()<<CYANB<<"\ncaller context label:"<<context_inst_pairs.first.first<<RST;
//
//                    BasicBlock *bb=context_inst_pairs.first.second->getParent();
//                    pair<int,BasicBlock*>context_bb_pair=make_pair(context_inst_pairs.first.first,bb);
//                    if(!forward_worklist_contains_this_entry[context_bb_pair])
//                    {
//                        forward_worklist.push(context_bb_pair);
//                        forward_worklist_contains_this_entry[context_bb_pair]=true;
//                    }
//                    if(!backward_worklist_contains_this_entry[context_bb_pair])
//                    {
//                        backward_worklist.push(context_bb_pair);
//                        backward_worklist_contains_this_entry[context_bb_pair]=true;
//                    }
//                    // forward_worklist.push(context_bb_pair);
//                    // backward_worklist.push(context_bb_pair);
//
//                }
//            }
//
//        }
//
//        // llvm::outs()<<WHITEB<<"\nBB_IN"<<CS_BB_IN[current_pair].second[0]<<RST;
//    }
//    // else
//    // {
//        //worklist is empty
//        llvm::outs()<<GREENB"\nNothing left to process in backward direction."<<RST;
//    // }
//}

//template <class F,class B>
//B Analysis<F,B>::NormalFlowFunctionBackward(pair<int,BasicBlock*> current_pair_of_context_label_and_bb)
//{
//    llvm::outs() << "AYUSH" << "\n";
//    BasicBlock &b=*(current_pair_of_context_label_and_bb.second);
//    B prev=CS_BB_OUT[current_pair_of_context_label_and_bb].second;
//    F prev_f = CS_BB_OUT[current_pair_of_context_label_and_bb].first;
//    //traverse a basic block in backward direction
//    // for(auto inst=b.rbegin();inst!=b.rend();inst++)
//    for(auto inst=&*(b.rbegin());inst!=NULL;inst=inst->getPrevNonDebugInstruction())
//    {
//        llvm::outs() << "===================================BACKWARD===========================================" << "\n";
//        llvm::outs() << "INSTRUCTION: " << *inst << "\n";
//        // llvm::outs()<<"\n"<<*inst;
//        // OUT[&(*inst)].second=prev;//compute OUT from previous IN-value
//        setBackwardComponentAtOutOfThisInstruction(&(*inst),prev);//compute OUT from previous IN-value
//        // IN[&(*inst)].second=computeInFromOut(*inst);
//        llvm::outs() << "OUT: ";
//        llvm::outs() << "\n";
//        printDataFlowValuesForward(prev_f);
//        printDataFlowValuesBackward(prev);
//        llvm::outs() << "\n";
//        B new_dfv = computeInFromOut(*inst);
//        setBackwardComponentAtInOfThisInstruction(&(*inst),new_dfv);
//        // prev=IN[&(*inst)].second;
//        llvm::outs() << "IN: " << "\n";
//        printDataFlowValuesForward(prev_f);
//        printDataFlowValuesBackward(new_dfv);
//        llvm::outs() << "\n";
//        prev=getBackwardComponentAtInOfThisInstruction(*inst);
//        // llvm::outs()<<MAGENTAB"\nnormal prev:";
//        // for(auto x:prev)
//        // {
//        //     llvm::outs()<<"\t"<<x->getName();
//        // }
//        // llvm::outs()<<"\n";
//        // llvm::outs()<<RST;
//        llvm::outs() << "===================================BACKWARD===========================================" << "\n";
//    }
//    return prev;
//}



template <class F,class B>
void Analysis<F,B>::performSplittingBB(Function &function)
{
    int flag=0;
    Instruction *I=NULL;
    bool previousInstructionWasSplitted=false;
    bool previousInstructionWasCallInstruction=false;
    map<Instruction *,bool>isSplittedOnThisInstruction;
    for(BasicBlock *BB:inverse_post_order(&function.back()))
    {
        BasicBlock &b=*BB;
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
                    if(not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(I))
                    {
                        continue;//this is an inbuilt function so doesn't need to be processed.
                    }
                    isSplittedOnThisInstruction[I]=false;
                    previousInstructionWasCallInstruction=true;
                    previousInstructionWasSplitted=true;//creating a false positive
                }
                continue;    
            }

            if(isa<BranchInst>(*I))
            {
                
            }
            else if (isa<CallInst>(*I))
            {
                CallInst *ci = dyn_cast<CallInst>(I);
                
        
                Function* target_function= ci->getCalledFunction();
                if(not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(I))
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
        if(split_here.second==false)//no splitting to be done
            continue;
        containingBB=split_here.first->getParent();
        containingBB->splitBasicBlock(split_here.first);
    }
    
}

template<class F,class B>
void Analysis<F,B>::freeMemory(int context_label){
    if(isFree[context_label]){
        return;
    }
    CS_BB_IN.erase(context_label);
    CS_BB_OUT.erase(context_label);
    isFree[context_label] = true;
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
    llvm::outs() << "\n";
    for(auto label : ProcedureContext){
        llvm::outs() << "==================================================================================================" << "\n";
        auto context = context_label_to_context_object_map[label];
        llvm::outs() << "LABEL: " << label << "\n";
        llvm::outs() << "FUNCTION NAME: " << context->getFunction()->getName() << "\n";
        llvm::outs() << "INFLOW VALUE: ";
        printDataFlowValuesForward(context->getInflowValue().first);
        printDataFlowValuesBackward(context->getInflowValue().second);
        llvm::outs() << "OUTFLOW VALUE: ";
        printDataFlowValuesForward(context->getOutflowValue().first);
        printDataFlowValuesBackward(context->getOutflowValue().second);
        llvm::outs() << "\n";
        llvm::outs() << "==================================================================================================" << "\n";
    }
}

template <class F,class B>
void Analysis<F,B>::printInOutMaps() {
//    llvm::outs() << "Printing IN-OUT maps:-\n";
//    for(auto p : IN){
//        llvm::outs() << p.first.first << " : ";
//        llvm::outs() << *p.first.second << " -> ";
//        printDataFlowValuesForward(p.second.first);
//    }
}






#endif

