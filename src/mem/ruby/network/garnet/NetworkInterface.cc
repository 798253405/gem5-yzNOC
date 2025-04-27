/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * Copyright (c) 2020 Inria
 * Copyright (c) 2016 Georgia Institute of Technology
 * Copyright (c) 2008 Princeton University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "mem/ruby/network/garnet/NetworkInterface.hh"

#include <cassert>
#include <cmath>

#include "base/cast.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet/Credit.hh"
#include "mem/ruby/network/garnet/flitBuffer.hh"
#include "mem/ruby/slicc_interface/Message.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{
int NetworkInterface::totalWrittenNIs = 0;
NetworkInterface::NetworkInterface(const Params &p)
  : ClockedObject(p), Consumer(this), m_id(p.id),
    m_virtual_networks(p.virt_nets), m_vc_per_vnet(0),
    m_vc_allocator(m_virtual_networks, 0),
    m_deadlock_threshold(p.garnet_deadlock_threshold),
    vc_busy_counter(m_virtual_networks, 0),
    m_yztick_event([this]{ yzperTickFunction(); },  // 初始化事件
                    name() + ".perTickEvent",
                    false,
                    Event::Progress_Event_Pri),
    m_yzRecordSelfInjPacket([this]{ m_yzRecordSelfInjPacketFunction(); },  // 初始化事件
                    name() + ".m_yzRecordSelfInjPacket",
                    false,
                    Event::Progress_Event_Pri)
                    
{
    m_stall_count.resize(m_virtual_networks);
    niOutVcs.resize(0);
   

   


 #ifdef yz250218RLReadFile
    // 读取 action 文件，获取 episode 编号
    std::string action_filename = "yzRLPython/logs/action_" + std::to_string(m_id) + ".txt";
    std::ifstream action_infile(action_filename);
    // 读取最新的 Tick 和 Value
    std::string filenameCPPRead = "yzRLPython/logs/action_" + std::to_string(2025) + ".txt";//m_id
    std::ifstream infile(filenameCPPRead);
   if (infile.is_open()) {
       std::string lastLine;
       std::string line;
       // **循环读取直到文件末尾，确保读取最后一行**
       while (std::getline(infile, line)) {
           lastLine = line;
       }
       infile.close();
       if (!lastLine.empty()) {
           std::istringstream iss(lastLine);
           iss  >> pythonReadTick >>  yzActionFromPython;  // **读取 tick 和 action 值**
           pythonReadTick = 0; // 重置为 0
           yzActionFromPython = 1;
       }
       
   }
   #endif
}













float NetworkInterface::yz_shareActionAllNIs = 1.0f;
float NetworkInterface::yz_shareNICPURequestList[128] = {0.0f};
float NetworkInterface::yz_shareInjRateNoC = 0.0f;
float NetworkInterface::yz_shareNoCTotalPacketCount = 0.0f;
void NetworkInterface::yzperTickFunction()
{
    // 1. 打开文件，写入当前 NI 的 Tick
     // 不管是不是第16个ni，都要执行下一次,重新调度下一次事件
     schedule(m_yztick_event, clockEdge(Cycles(yzResetTokenPeriod)));
        
     if (   newBashEnable == true )   {  //newBashEnable == true 一开始就启动。  //curTick()>  (get_max_tick() - 5*yzResetTokenPeriod*500) &&  newBashEnable == true  可以跑很多天但是一开始慢
        newBashEnable = false; // 防止重复执行
        if (m_id == 0) {
            std::string command = 
                "gnome-terminal -- bash -c '"
                "pwd; ls; "
                //"./build/X86_MOESI_hammeryz1point0/gem5.opt "
            "./build/X86_MOESI_hammeryz1wVCBuffer/gem5.opt "
                "-d m5out/250225/blacksholes/"+std::to_string(get_max_tick()) +"/"
                //"-d m5out/250225/bodytrack/ "
                " configs/deprecated/example/fs.py "
                "--checkpoint-restore=1 "
                "--checkpoint-dir=/home/yz/myprojects/2024GEM5/parsec-tests/yzmodifiedgem5/m5out/checkpoint/250421 "
                "--kernel=/home/yz/.cache/gem5/x86-linux-kernel-4.19.83 "
                "--disk=/home/yz/.cache/gem5/x86-parsec "
                "--restore-with-cpu=AtomicSimpleCPU "
                "--cpu-type=X86O3CPU "//X86TimingSimpleCPU
                "--num-cpus=64 "
                "--ruby "
                "--network=garnet "
                "--topology=Mesh_XY "
                "--mesh-rows=8 "
                "--num-dirs=64 "
                "--num-l2caches=64 "
                "--script=configs/yz2023Nov/large_directparsec/yzfs_largeparsecblacksholes.script "
                //"--script=configs/yz2023Nov/large_directparsec/yzfs_largeparsecbodytrack.script "
                "--abs-max-tick=" + std::to_string(get_max_tick() + yzResetTokenPeriod * 500) +
                "; '";   

            system(command.c_str());  // 运行 shell 命令
            }
        }


    // 更新已写入的 NI 数量
    totalWrittenNIs++;


    int tempHighestInjRate = 0;
    int tempLowestInjRate =0;
    for(int i = 0; i < 128; i++){
        yz_shareNICPURequestList[m_id] = yzPeriodActualInjPacketCount ;//
        }
    for(int i = 0; i < 64; i++){
   
        if (yz_shareNICPURequestList[m_id] > tempHighestInjRate){
            tempHighestInjRate = yz_shareNICPURequestList[m_id];
        }
        if (yz_shareNICPURequestList[m_id] < tempLowestInjRate){
            tempLowestInjRate = yz_shareNICPURequestList[m_id];
        }
    }

     tempThreshold = int((tempLowestInjRate + (tempHighestInjRate-tempLowestInjRate)*0.7));   

     DPRINTF(yzzzzNI, "yzperTickFunction() AT %lu\n", curTick()); // 可選的調試信息

   // if(curTick()>  (get_max_tick() - 20*yzResetTokenPeriod*500)  ) 
    {
    std::stringstream filename_oneAction;
    filename_oneAction << "yzRLPython/oneAction/inj_"<<NetworkInterface::yz_shareActionAllNIs<<"/"<<get_max_tick()<<"_node_" <<m_id<< ".txt"; //m_id // 注意，每次的slowest m_id都可能不一样？
    std::string filename_oneActionCPPWrite = filename_oneAction.str(); // 转换为 std::string
    std::ofstream outfile2(filename_oneActionCPPWrite, std::ios::app);
    if (outfile2.is_open()) {
        outfile2  << curTick()   <<" , yz_shareNICPURequestList[m_id] "<<yz_shareNICPURequestList[m_id] <<" ,  tempThreshold  "<< tempThreshold  <<" , yzPeriodActualInjPacketCount "<<yzPeriodActualInjPacketCount
        <<" , yz_InjRate "<<yz_InjRate <<" , yz_ADNewPeriodtokenGenerated "<<yz_ADNewPeriodtokenGenerated
         <<" , yzPacketPeriodAvgQueueDelay  "<<yzPacketPeriodAvgQueueDelay  <<" , yzPacketPeriodAvgNetDelay  "<<yzPacketPeriodAvgNetDelay  <<" , yzPacketPeriodCountreceived " <<yzPacketPeriodCount
        <<" \n";
        outfile2.close();
    } else {
        warn("NetworkInterface::yzperTickFunction(): could not open yzRLPython/oneAction/ NetworkInterface::yz_shareActionAllNIs\n");
    }

    }


 
    
    uint64_t tempMsgCounter;
    if (inNode_ptr.size() >=2  ) 
    {   for(int t_vnet = 0; t_vnet < m_virtual_networks; t_vnet++){
            MessageBuffer *b = inNode_ptr[t_vnet];
            if (b == nullptr){
             
            }
            else
            {
            tempMsgCounter = tempMsgCounter + b->m_msg_counter;
            }
    }
    }
     
    yz_curCPUInjSignalCount = tempMsgCounter;
    yz_preCPUInjSignalCount = yz_curCPUInjSignalCount;


         
 
  if(m_id == 0){
       //yzReadAndStuckForPythonFIle(m_id); // 读取 Python 文件，等待 Python 更新 Tick
       //DPRINTF(yzzzzNI, "stcuk ends line256 atTick %lld cycle %lld \n",curTick() ,curCycle() );
        }
   
     
}


//yzkth
void NetworkInterface::m_yzRecordSelfInjPacketFunction(){
    schedule(m_yzRecordSelfInjPacket, clockEdge(Cycles(yzResetTokenPeriod)));

    //tempThreshold = 73;
/*
    //调控强度
    #ifdef yz250218RLReadFile
    if(yzPacketPeriodAvgQueueDelay > 10){
        NetworkInterface::yz_shareActionAllNIs= 1.0;
    }
    else if(float(yzPeriodActualInjPacketCount)> 300)
    //python 计算更新
    // NetworkInterface::yz_shareActionAllNIs= float(yzPeriodActualInjPacketCount) / float(yzResetTokenPeriod) *  yzActionFromPython;//
    //手动rule更新
    NetworkInterface::yz_shareActionAllNIs= float(yzPeriodActualInjPacketCount) / float(yzResetTokenPeriod) * 0.8;//yzActionFromPython;//
    else{
        NetworkInterface::yz_shareActionAllNIs= 1.0;
    }
     #endif
*/
NetworkInterface::yz_shareActionAllNIs= 0.4; //1.2 for bodytrack
 

    //./build/X86_MOESI_hammeryz1wVCBuffer/gem5.opt --debug-flags=yzzzzNI   -d m5out/250225/blacksholes/ configs/deprecated/example/fs.py     --checkpoint-restore=1  --checkpoint-dir=/home/yz/myprojects/2024GEM5/parsec-tests/yzmodifiedgem5/m5out/checkpoint/250224  --kernel=/home/yz/.cache/gem5/x86-linux-kernel-4.19.83 --disk=/home/yz/.cache/gem5/x86-parsec   --restore-with-cpu=AtomicSimpleCPU    --cpu-type=X86TimingSimpleCPU     --num-cpus=64   --ruby   --network=garnet   --topology=Mesh_XY   --mesh-rows=8 --num-dirs=64  --num-l2caches=64  --script=configs/yz2023Nov/large/yzfs_largeparsecblacksholes.script --abs-max-tick=334516476158500
 
    if (yzPeriodActualInjPacketCount  >  tempThreshold  && curTick()==  (get_max_tick() - 20*yzResetTokenPeriod*500)  ) { //
        yz_InjRate = NetworkInterface::yz_shareActionAllNIs * float(yzPeriodActualInjPacketCount) / float(yzResetTokenPeriod) ;//调控，但是只调一个period
        //yz_InjRate = NetworkInterface::yz_shareActionAllNIs  * NetworkInterface::yz_shareNoCTotalPacketCount / float(yzResetTokenPeriod)  / float(64); //调控，但是只调一个period  而且全部节点统一
        if(NetworkInterface::yz_shareActionAllNIs > 0.98){
            yz_InjRate = 1;
        }
        else if (m_id >63){
            yz_InjRate = 1; //64-127 不调控
        }
        // yz_InjRate = 1.0f; // no flow regulation
        }
    else{
       yz_InjRate = 1.0f; // no flow regulation
    }


 
    


    

   // reset the record of each period state
   yzPacketPeriodSumQueueDelay = 0;
   yzPacketPeriodSumNetDelay = 0;
   yzPacketPeriodCount = 0;
   yzPacketPeriodAvgQueueDelay = 0;
   yzPacketPeriodAvgNetDelay = 0;
  // DPRINTF(RubyNetwork, " at time: %lld yz totalWrittenNIs:%d m_id  %d\n",curTick(), totalWrittenNIs, m_id);

    yzResetBucketPeriod();  

}



void NetworkInterface::yzResetBucketPeriod(){
 
    yzPeriodActualInjPacketCount = 0;//   重置为 0
    //reset
    yz_ADtokenWasted =0;
    
    yz_ADtokenGenerated = 0;
    yz_ADtokenUsed = 0;
    yz_tokenInBucket = 5; //reset to  be 1 token, avoid ->0 is too sharp
    
    
    yzLastPeriodCycleForTokenGen = curCycle();
    yzLast_ADNewPeriodtokenGenerated = 0;
    yz_ADNewPeriodtokenGenerated = 0;

    if(m_id == 0){
        NetworkInterface::yz_shareInjRateNoC = 0;
        NetworkInterface::yz_shareNoCTotalPacketCount = 0;
    }
   
}




void NetworkInterface::yzReadAndStuckForPythonFIle(int in_m_id){
    int m_id = in_m_id;
 
    // 3. 读取 等待 Python 更新 Tick
    while (true) { //true
       

        // 读取最新的 Tick 和 Value
        std::string filenameCPPRead = "yzRLPython/logs/action_" + std::to_string(2025) + ".txt";//m_id
         std::ifstream infile(filenameCPPRead);
        if (infile.is_open()) {
            std::string lastLine;
            std::string line;
            // **循环读取直到文件末尾，确保读取最后一行**
            while (std::getline(infile, line)) {
                lastLine = line;
            }
            infile.close();
            if (!lastLine.empty()) {
                std::istringstream iss(lastLine);
                iss  >> pythonReadTick >>  yzActionFromPython;  // **读取 tick 和 action 值**
            }
        }
 
        DPRINTF(yzzzzNI, "I am always read tick   curTick()-pythonReadTick %lld  yzActionFromPython%f  \n",  pythonReadTick- curTick() ,yzActionFromPython ) ;
       //intpythonValue = 25 
        // 如果 Tick 发生变化
        if (pythonReadTick >= curTick()) {// 说明 Python 已经更新了文件
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 10ms 休眠，减少 CPU 占用
        
    }
}

void
NetworkInterface::addInPort(NetworkLink *in_link,
                              CreditLink *credit_link)
{
    InputPort *newInPort = new InputPort(in_link, credit_link);
    inPorts.push_back(newInPort);
    DPRINTF(RubyNetwork, "Adding input port:%s with vnets %s\n",
    in_link->name(), newInPort->printVnets());

    in_link->setLinkConsumer(this);
    credit_link->setSourceQueue(newInPort->outCreditQueue(), this);
    if (m_vc_per_vnet != 0) {
        in_link->setVcsPerVnet(m_vc_per_vnet);
        credit_link->setVcsPerVnet(m_vc_per_vnet);
    }

}

void
NetworkInterface::addOutPort(NetworkLink *out_link,
                             CreditLink *credit_link,
                             SwitchID router_id, uint32_t consumerVcs)
{
    OutputPort *newOutPort = new OutputPort(out_link, credit_link, router_id);
    outPorts.push_back(newOutPort);

    assert(consumerVcs > 0);
    // We are not allowing different physical links to have different vcs
    // If it is required that the Network Interface support different VCs
    // for every physical link connected to it. Then they need to change
    // the logic within outport and inport.
    if (niOutVcs.size() == 0) {
        m_vc_per_vnet = consumerVcs;
        int m_num_vcs = consumerVcs * m_virtual_networks;
        niOutVcs.resize(m_num_vcs);
        outVcState.reserve(m_num_vcs);
        m_ni_out_vcs_enqueue_time.resize(m_num_vcs);
        // instantiating the NI flit buffers
        for (int i = 0; i < m_num_vcs; i++) {
            m_ni_out_vcs_enqueue_time[i] = Tick(INFINITE_);
            outVcState.emplace_back(i, m_net_ptr, consumerVcs);
        }

        // Reset VC Per VNET for input links already instantiated
        for (auto &iPort: inPorts) {
            NetworkLink *inNetLink = iPort->inNetLink();
            inNetLink->setVcsPerVnet(m_vc_per_vnet);
            credit_link->setVcsPerVnet(m_vc_per_vnet);
        }
    } else {
        fatal_if(consumerVcs != m_vc_per_vnet,
        "%s: Connected Physical links have different vc requests: %d and %d\n",
        name(), consumerVcs, m_vc_per_vnet);
    }

    DPRINTF(RubyNetwork, "OutputPort:%s Vnet: %s\n",
    out_link->name(), newOutPort->printVnets());

    out_link->setSourceQueue(newOutPort->outFlitQueue(), this);
    out_link->setVcsPerVnet(m_vc_per_vnet);
    credit_link->setLinkConsumer(this);
    credit_link->setVcsPerVnet(m_vc_per_vnet);
}

void
NetworkInterface::addNode(std::vector<MessageBuffer *>& in,
                          std::vector<MessageBuffer *>& out)
{
    inNode_ptr = in;
    outNode_ptr = out;

    for (auto& it : in) {
        if (it != nullptr) {
            it->setConsumer(this);
        }
    }
}

void
NetworkInterface::dequeueCallback()
{
    // An output MessageBuffer has dequeued something this cycle and there
    // is now space to enqueue a stalled message. However, we cannot wake
    // on the same cycle as the dequeue. Schedule a wake at the soonest
    // possible time (next cycle).
    scheduleEventAbsolute(clockEdge(Cycles(1)));
}

void
NetworkInterface::incrementStats(flit *t_flit)
{
    int vnet = t_flit->get_vnet();

    // Latency
    m_net_ptr->increment_received_flits(vnet);
    Tick network_delay =
        t_flit->get_dequeue_time() -
        t_flit->get_enqueue_time() - cyclesToTicks(Cycles(1));
    Tick src_queueing_delay = t_flit->get_src_delay();
    Tick dest_queueing_delay = (curTick() - t_flit->get_dequeue_time());
    Tick queueing_delay = src_queueing_delay + dest_queueing_delay;

    m_net_ptr->increment_flit_network_latency(network_delay, vnet);
    m_net_ptr->increment_flit_queueing_latency(queueing_delay, vnet);

    if (t_flit->get_type() == TAIL_ || t_flit->get_type() == HEAD_TAIL_) {
        m_net_ptr->increment_received_packets(vnet);
        m_net_ptr->increment_packet_network_latency(network_delay, vnet);
        m_net_ptr->increment_packet_queueing_latency(queueing_delay, vnet);
        m_net_ptr->increment_yzAllreceivedpackets(vnet,network_delay,queueing_delay);
        //加一个在queue的 message buffer
        //yzKTH int src_ni_id = t_flit->get_route().src_ni;
        yzOneNI_recordOnePacket(t_flit->get_route().src_ni ,t_flit->get_route().dest_ni, m_id, vnet,queueing_delay, network_delay);
    }

    // Hops
    m_net_ptr->increment_total_hops(t_flit->get_route().hops_traversed);
}
void NetworkInterface::yzOneNI_recordOnePacket(int  sourceNIID, int dest_niID,int recvNIID  , int onWhichVNet,float in_queueing_delay,  float in_network_delay) {
    yzPacketPeriodSumQueueDelay = yzPacketPeriodSumQueueDelay + in_queueing_delay;
    yzPacketPeriodSumNetDelay  =  yzPacketPeriodSumNetDelay + in_network_delay;
    yzPacketPeriodCount  =  yzPacketPeriodCount  + 1;
    yzPacketPeriodAvgQueueDelay = yzPacketPeriodSumQueueDelay /500 ; //   / yzPacketPeriodCount
    yzPacketPeriodAvgNetDelay = yzPacketPeriodSumNetDelay /500; //   / yzPacketPeriodCount 

     
}

/*
 * The NI wakeup checks whether there are any ready messages in the protocol
 * buffer. If yes, it picks that up, flitisizes it into a number of flits and
 * puts it into an output buffer and schedules the output link. On a wakeup
 * it also checks whether there are flits in the input link. If yes, it picks
 * them up and if the flit is a tail, the NI inserts the corresponding message
 * into the protocol buffer. It also checks for credits being sent by the
 * downstream router.
 */

void
NetworkInterface::wakeup()
{  

    if( yzCheckIniEvent == 0){ // FIRST TIME
        yzCheckIniEvent = 1;    
        #ifdef  yzRecordActualInjRate       
        //yzkth
        schedule(m_yztick_event, (get_max_tick() - 30*yzResetTokenPeriod*500) ); // 首次调度事件. 解藕统计state和更新action.
        schedule( m_yzRecordSelfInjPacket, (get_max_tick() - 30*yzResetTokenPeriod*500) ); // 首次调度事件    
        #endif
        
    }


    std::ostringstream oss;
    for (auto &oPort: outPorts) {
        oss << oPort->routerID() << "[" << oPort->printVnets() << "] ";
    }
   
    assert(curTick() == clockEdge());
    MsgPtr msg_ptr;
    Tick curTime = clockEdge();
 
    // Checking for messages coming from the protocol
    // can pick up a message/cycle for each virtual net
    for (int vnet = 0; vnet < inNode_ptr.size(); ++vnet) {
        MessageBuffer *b = inNode_ptr[vnet];
        if (b == nullptr) {
            continue;
        }

        if (b->isReady(curTime)) { // Is there a message waiting
            msg_ptr = b->peekMsgPtr();
            //std::cout<<"coutdebugyzzzznetworkinterfaceline214"<<"NI::wakeup()_msg_ptr "<<msg_ptr.get()<<" curTick()is "<<curTick()<<std::endl;
            #ifdef  yz250203LeakyBucketOn
            if (yzModifiedflitisizeMessage(msg_ptr, vnet))
            #else
            if (flitisizeMessage(msg_ptr, vnet))
            #endif 
            {
                b->dequeue(curTime);
                yzPeriodActualInjPacketCount++;
                yz_shareNoCTotalPacketCount++;
            }
        }
    }

    scheduleOutputLink();

    // Check if there are flits stalling a virtual channel. Track if a
    // message is enqueued to restrict ejection to one message per cycle.
    checkStallQueue();

    /*********** Check the incoming flit link **********/
    DPRINTF(RubyNetwork, "Number of input ports: %d\n", inPorts.size());
    for (auto &iPort: inPorts) {
        NetworkLink *inNetLink = iPort->inNetLink();
        if (inNetLink->isReady(curTick())) {
            flit *t_flit = inNetLink->consumeLink();
            DPRINTF(RubyNetwork, "Recieved flit:%s\n", *t_flit);
            assert(t_flit->m_width == iPort->bitWidth());

            int vnet = t_flit->get_vnet();
            t_flit->set_dequeue_time(curTick());

            // If a tail flit is received, enqueue into the protocol buffers
            // if space is available. Otherwise, exchange non-tail flits for
            // credits.
            if (t_flit->get_type() == TAIL_ ||
                t_flit->get_type() == HEAD_TAIL_) {
                if (!iPort->messageEnqueuedThisCycle &&
                    outNode_ptr[vnet]->areNSlotsAvailable(1, curTime)) {
                    // Space is available. Enqueue to protocol buffer.
                    outNode_ptr[vnet]->enqueue(t_flit->get_msg_ptr(), curTime,
                                               cyclesToTicks(Cycles(1)));
                    DPRINTF(RubyNetwork, "debugyzzzzselfAdded Recieved tailflit:%s msg=%s\n", *t_flit, t_flit->get_msg_ptr());
                    // Simply send a credit back since we are not buffering
                    // this flit in the NI
                    Credit *cFlit = new Credit(t_flit->get_vc(),
                                               true, curTick());
                    iPort->sendCredit(cFlit);
                    // Update stats and delete flit pointer
                    incrementStats(t_flit);
                    delete t_flit;
                } else {
                    // No space available- Place tail flit in stall queue and
                    // set up a callback for when protocol buffer is dequeued.
                    // Stat update and flit pointer deletion will occur upon
                    // unstall.
                    iPort->m_stall_queue.push_back(t_flit);
                    m_stall_count[vnet]++;

                    outNode_ptr[vnet]->registerDequeueCallback([this]() {
                        dequeueCallback(); });
                }
            } else {
                // Non-tail flit. Send back a credit but not VC free signal.
                Credit *cFlit = new Credit(t_flit->get_vc(), false,
                                               curTick());
                // Simply send a credit back since we are not buffering
                // this flit in the NI
                iPort->sendCredit(cFlit);

                // Update stats and delete flit pointer.
                incrementStats(t_flit);
                delete t_flit;
            }
        }
    }

    /****************** Check the incoming credit link *******/

    for (auto &oPort: outPorts) {
        CreditLink *inCreditLink = oPort->inCreditLink();
        if (inCreditLink->isReady(curTick())) {
            Credit *t_credit = (Credit*) inCreditLink->consumeLink();
            outVcState[t_credit->get_vc()].increment_credit();
            if (t_credit->is_free_signal()) {
                outVcState[t_credit->get_vc()].setState(IDLE_,
                    curTick());
            }
            delete t_credit;
        }
    }


    // It is possible to enqueue multiple outgoing credit flits if a message
    // was unstalled in the same cycle as a new message arrives. In this
    // case, we should schedule another wakeup to ensure the credit is sent
    // back.
    for (auto &iPort: inPorts) {
        if (iPort->outCreditQueue()->getSize() > 0) {
            DPRINTF(RubyNetwork, "Sending a credit %s via %s at %ld\n",
            *(iPort->outCreditQueue()->peekTopFlit()),
            iPort->outCreditLink()->name(), clockEdge(Cycles(1)));
            iPort->outCreditLink()->
                scheduleEventAbsolute(clockEdge(Cycles(1)));
        }
    }
    checkReschedule();
}

void
NetworkInterface::checkStallQueue()
{
    // Check all stall queues.
    // There is one stall queue for each input link
    for (auto &iPort: inPorts) {
        iPort->messageEnqueuedThisCycle = false;
        Tick curTime = clockEdge();

        if (!iPort->m_stall_queue.empty()) {
            for (auto stallIter = iPort->m_stall_queue.begin();
                 stallIter != iPort->m_stall_queue.end(); ) {
                flit *stallFlit = *stallIter;
                int vnet = stallFlit->get_vnet();

                // If we can now eject to the protocol buffer,
                // send back credits
                if (outNode_ptr[vnet]->areNSlotsAvailable(1,
                    curTime)) {
                    outNode_ptr[vnet]->enqueue(stallFlit->get_msg_ptr(),
                        curTime, cyclesToTicks(Cycles(1)));

                    // Send back a credit with free signal now that the
                    // VC is no longer stalled.
                    Credit *cFlit = new Credit(stallFlit->get_vc(), true,
                                                   curTick());
                    iPort->sendCredit(cFlit);

                    // Update Stats
                    incrementStats(stallFlit);

                    // Flit can now safely be deleted and removed from stall
                    // queue
                    delete stallFlit;
                    iPort->m_stall_queue.erase(stallIter);
                    m_stall_count[vnet]--;

                    // If there are no more stalled messages for this vnet, the
                    // callback on it's MessageBuffer is not needed.
                    if (m_stall_count[vnet] == 0)
                        outNode_ptr[vnet]->unregisterDequeueCallback();

                    iPort->messageEnqueuedThisCycle = true;
                    break;
                } else {
                    ++stallIter;
                }
            }
        }
    }
}

// Embed the protocol message into flits
bool
NetworkInterface::flitisizeMessage(MsgPtr msg_ptr, int vnet)
{
    Message *net_msg_ptr = msg_ptr.get();
    NetDest net_msg_dest = net_msg_ptr->getDestination();

    // gets all the destinations associated with this message.
    std::vector<NodeID> dest_nodes = net_msg_dest.getAllDest();

    // Number of flits is dependent on the link bandwidth available.
    // This is expressed in terms of bytes/cycle or the flit size
    OutputPort *oPort = getOutportForVnet(vnet);
    assert(oPort);
    int num_flits = (int)divCeil((float) m_net_ptr->MessageSizeType_to_int(
        net_msg_ptr->getMessageSize()), (float)oPort->bitWidth());

    DPRINTF(RubyNetwork, "Message Size:%d vnet:%d bitWidth:%d\n",
        m_net_ptr->MessageSizeType_to_int(net_msg_ptr->getMessageSize()),
        vnet, oPort->bitWidth());

    // loop to convert all multicast messages into unicast messages
    for (int ctr = 0; ctr < dest_nodes.size(); ctr++) {

        // this will return a free output virtual channel
        int vc = calculateVC(vnet);

        if (vc == -1) {
            return false ;
        }
        MsgPtr new_msg_ptr = msg_ptr->clone();
        NodeID destID = dest_nodes[ctr];

        Message *new_net_msg_ptr = new_msg_ptr.get();
        if (dest_nodes.size() > 1) {
            NetDest personal_dest;
            for (int m = 0; m < (int) MachineType_NUM; m++) {
                if ((destID >= MachineType_base_number((MachineType) m)) &&
                    destID < MachineType_base_number((MachineType) (m+1))) {
                    // calculating the NetDest associated with this destID
                    personal_dest.clear();
                    personal_dest.add((MachineID) {(MachineType) m, (destID -
                        MachineType_base_number((MachineType) m))});
                    new_net_msg_ptr->getDestination() = personal_dest;
                    break;
                }
            }
            net_msg_dest.removeNetDest(personal_dest);
            // removing the destination from the original message to reflect
            // that a message with this particular destination has been
            // flitisized and an output vc is acquired
            net_msg_ptr->getDestination().removeNetDest(personal_dest);
        }

        // Embed Route into the flits
        // NetDest format is used by the routing table
        // Custom routing algorithms just need destID

        RouteInfo route;
        route.vnet = vnet;
        route.net_dest = new_net_msg_ptr->getDestination();
        route.src_ni = m_id;
        route.src_router = oPort->routerID();
        route.dest_ni = destID;
        route.dest_router = m_net_ptr->get_router_id(destID, vnet);

        // initialize hops_traversed to -1
        // so that the first router increments it to 0
        route.hops_traversed = -1;

        m_net_ptr->increment_injected_packets(vnet);
        m_net_ptr->update_traffic_distribution(route);
        int packet_id = m_net_ptr->getNextPacketID();
        for (int i = 0; i < num_flits; i++) {
            m_net_ptr->increment_injected_flits(vnet);
            flit *fl = new flit(packet_id,
                i, vc, vnet, route, num_flits, new_msg_ptr,
                m_net_ptr->MessageSizeType_to_int(
                net_msg_ptr->getMessageSize()),
                oPort->bitWidth(), curTick());

            fl->set_src_delay(curTick() - msg_ptr->getTime());
            niOutVcs[vc].insert(fl);
        }

        m_ni_out_vcs_enqueue_time[vc] = curTick();
        outVcState[vc].setState(ACTIVE_, curTick());
    }
    return true ;
}
// Embed the protocol message into flits
bool
NetworkInterface::yzModifiedflitisizeMessage(MsgPtr msg_ptr, int vnet)
{   
    
    if(yz_InjRate > 0.99){
        yz_tokenInBucket  =  20000; //手动设置成一个好辨认的值
        yz_ADNewPeriodtokenGenerated = 20000;
    }
    else{
        yz_ADNewPeriodtokenGenerated = yzResetTokenPeriod * yz_InjRate*2;
        long long time_elapsed = curCycle() - yzLastPeriodCycleForTokenGen;
        if (time_elapsed > 0) {
            // 计算应该添加的令牌数
            double tokens_to_add = time_elapsed * yz_InjRate*2; //* 2 看看是不是原版的就即使100%比较小，所以
            // 更新当前令牌数，但不超过桶容量
            yz_tokenInBucket = std::min(5.0, yz_tokenInBucket + tokens_to_add); //BUCKET_CAPACITY,
            // 更新上次更新时间戳
            yzLastPeriodCycleForTokenGen = curCycle();
            if(yz_tokenInBucket <10000 && m_id == 0){
             //DPRINTF(yzzzzNI, "Tokens updated: Added %.2f, Current %.2f\n", tokens_to_add, yz_tokenInBucket );
            }
        } 
    }


 
    Message *net_msg_ptr = msg_ptr.get();
    NetDest net_msg_dest = net_msg_ptr->getDestination();

    // gets all the destinations associated with this message.
    std::vector<NodeID> dest_nodes = net_msg_dest.getAllDest();

    // Number of flits is dependent on the link bandwidth available.
    // This is expressed in terms of bytes/cycle or the flit size
    OutputPort *oPort = getOutportForVnet(vnet);
    assert(oPort);
    int num_flits = (int)divCeil((float) m_net_ptr->MessageSizeType_to_int(
        net_msg_ptr->getMessageSize()), (float)oPort->bitWidth());

    DPRINTF(RubyNetwork, "Message Size:%d vnet:%d bitWidth:%d\n",
        m_net_ptr->MessageSizeType_to_int(net_msg_ptr->getMessageSize()),
        vnet, oPort->bitWidth());

    // loop to convert all multicast messages into unicast messages
    for (int ctr = 0; ctr < dest_nodes.size(); ctr++) {

        // this will return a free output virtual channel
        int vc = calculateVC(vnet);

        if (vc == -1) {
            return false ;
        }

        if(yz_tokenInBucket < 1){
            return false;
         }
         else{
            yz_ADtokenUsed = yz_ADtokenUsed+1;
             yz_tokenInBucket = yz_tokenInBucket -1;
         }   
         
     
        MsgPtr new_msg_ptr = msg_ptr->clone();
        NodeID destID = dest_nodes[ctr];

        Message *new_net_msg_ptr = new_msg_ptr.get();
        if (dest_nodes.size() > 1) {
            NetDest personal_dest;
            for (int m = 0; m < (int) MachineType_NUM; m++) {
                if ((destID >= MachineType_base_number((MachineType) m)) &&
                    destID < MachineType_base_number((MachineType) (m+1))) {
                    // calculating the NetDest associated with this destID
                    personal_dest.clear();
                    personal_dest.add((MachineID) {(MachineType) m, (destID -
                        MachineType_base_number((MachineType) m))});
                    new_net_msg_ptr->getDestination() = personal_dest;
                    break;
                }
            }
            net_msg_dest.removeNetDest(personal_dest);
            // removing the destination from the original message to reflect
            // that a message with this particular destination has been
            // flitisized and an output vc is acquired
            net_msg_ptr->getDestination().removeNetDest(personal_dest);
        }

        // Embed Route into the flits
        // NetDest format is used by the routing table
        // Custom routing algorithms just need destID

        RouteInfo route;
        route.vnet = vnet;
        route.net_dest = new_net_msg_ptr->getDestination();
        route.src_ni = m_id;
        route.src_router = oPort->routerID();
        route.dest_ni = destID;
        route.dest_router = m_net_ptr->get_router_id(destID, vnet);

        // initialize hops_traversed to -1
        // so that the first router increments it to 0
        route.hops_traversed = -1;

        m_net_ptr->increment_injected_packets(vnet);
        m_net_ptr->yz_increment_injected_packets(vnet, m_id);// yz added //just record
        m_net_ptr->update_traffic_distribution(route);
        int packet_id = m_net_ptr->getNextPacketID();

       
        for (int i = 0; i < num_flits; i++) {
            m_net_ptr->increment_injected_flits(vnet);
            flit *fl = new flit(packet_id,
                i, vc, vnet, route, num_flits, new_msg_ptr,
                m_net_ptr->MessageSizeType_to_int(
                net_msg_ptr->getMessageSize()),
                oPort->bitWidth(), curTick());

            fl->set_src_delay(curTick() - msg_ptr->getTime());
            niOutVcs[vc].insert(fl);
            
        }

        m_ni_out_vcs_enqueue_time[vc] = curTick();
        outVcState[vc].setState(ACTIVE_, curTick());
    }
    return true ;
}

// Looking for a free output vc
int
NetworkInterface::calculateVC(int vnet)
{
    for (int i = 0; i < m_vc_per_vnet; i++) {
        int delta = m_vc_allocator[vnet];
        m_vc_allocator[vnet]++;
        if (m_vc_allocator[vnet] == m_vc_per_vnet)
            m_vc_allocator[vnet] = 0;

        if (outVcState[(vnet*m_vc_per_vnet) + delta].isInState(
                    IDLE_, curTick())) {
            vc_busy_counter[vnet] = 0;
            return ((vnet*m_vc_per_vnet) + delta);
        }
    }

    vc_busy_counter[vnet] += 1;
    panic_if(vc_busy_counter[vnet] > m_deadlock_threshold,
        "%s: Possible network deadlock in vnet: %d at time: %llu \n",
        name(), vnet, curTick());

    return -1;
}

void
NetworkInterface::scheduleOutputPort(OutputPort *oPort)
{
   int vc = oPort->vcRoundRobin();

   for (int i = 0; i < niOutVcs.size(); i++) {
       vc++;
       if (vc == niOutVcs.size())
           vc = 0;

       int t_vnet = get_vnet(vc);
       if (oPort->isVnetSupported(t_vnet)) {
           // model buffer backpressure
           if (niOutVcs[vc].isReady(curTick()) &&
               outVcState[vc].has_credit()) {

               bool is_candidate_vc = true;
               int vc_base = t_vnet * m_vc_per_vnet;

               if (m_net_ptr->isVNetOrdered(t_vnet)) {
                   for (int vc_offset = 0; vc_offset < m_vc_per_vnet;
                        vc_offset++) {
                       int t_vc = vc_base + vc_offset;
                       if (niOutVcs[t_vc].isReady(curTick())) {
                           if (m_ni_out_vcs_enqueue_time[t_vc] <
                               m_ni_out_vcs_enqueue_time[vc]) {
                               is_candidate_vc = false;
                               break;
                           }
                       }
                   }
               }
               if (!is_candidate_vc)
                   continue;

               // Update the round robin arbiter
               oPort->vcRoundRobin(vc);

               outVcState[vc].decrement_credit();

               // Just removing the top flit
               flit *t_flit = niOutVcs[vc].getTopFlit();
               t_flit->set_time(clockEdge(Cycles(1)));

               // Scheduling the flit
               scheduleFlit(t_flit);

               if (t_flit->get_type() == TAIL_ ||
                  t_flit->get_type() == HEAD_TAIL_) {
                   m_ni_out_vcs_enqueue_time[vc] = Tick(INFINITE_);
               }

               // Done with this port, continue to schedule
               // other ports
               return;
           }
       }
   }
}



/** This function looks at the NI buffers
 *  if some buffer has flits which are ready to traverse the link in the next
 *  cycle, and the downstream output vc associated with this flit has buffers
 *  left, the link is scheduled for the next cycle
 */

void
NetworkInterface::scheduleOutputLink()
{
    // Schedule each output link
    for (auto &oPort: outPorts) {
        scheduleOutputPort(oPort);
    }
}

NetworkInterface::InputPort *
NetworkInterface::getInportForVnet(int vnet)
{
    for (auto &iPort : inPorts) {
        if (iPort->isVnetSupported(vnet)) {
            return iPort;
        }
    }

    return nullptr;
}

/*
 * This function returns the outport which supports the given vnet.
 * Currently, HeteroGarnet does not support multiple outports to
 * support same vnet. Thus, this function returns the first-and
 * only outport which supports the vnet.
 */
NetworkInterface::OutputPort *
NetworkInterface::getOutportForVnet(int vnet)
{
    for (auto &oPort : outPorts) {
        if (oPort->isVnetSupported(vnet)) {
            return oPort;
        }
    }

    return nullptr;
}
void
NetworkInterface::scheduleFlit(flit *t_flit)
{
    OutputPort *oPort = getOutportForVnet(t_flit->get_vnet());

    if (oPort) {
        DPRINTF(RubyNetwork, "Scheduling at %s time:%ld flit:%s Message:%s\n",
        oPort->outNetLink()->name(), clockEdge(Cycles(1)),
        *t_flit, *(t_flit->get_msg_ptr()));
        oPort->outFlitQueue()->insert(t_flit);
        oPort->outNetLink()->scheduleEventAbsolute(clockEdge(Cycles(1)));
        return;
    }

    panic("No output port found for vnet:%d\n", t_flit->get_vnet());
    return;
}

int
NetworkInterface::get_vnet(int vc)
{
    for (int i = 0; i < m_virtual_networks; i++) {
        if (vc >= (i*m_vc_per_vnet) && vc < ((i+1)*m_vc_per_vnet)) {
            return i;
        }
    }
    fatal("Could not determine vc");
}


// Wakeup the NI in the next cycle if there are waiting
// messages in the protocol buffer, or waiting flits in the
// output VC buffer.
// Also check if we have to reschedule because of a clock period
// difference.
void
NetworkInterface::checkReschedule()
{
    for (const auto& it : inNode_ptr) {
        if (it == nullptr) {
            continue;
        }

        while (it->isReady(clockEdge())) { // Is there a message waiting
            scheduleEvent(Cycles(1));
            return;
        }
    }

    for (auto& ni_out_vc : niOutVcs) {
        if (ni_out_vc.isReady(clockEdge(Cycles(1)))) {
            scheduleEvent(Cycles(1));
            return;
        }
    }

    // Check if any input links have flits to be popped.
    // This can happen if the links are operating at
    // a higher frequency.
    for (auto &iPort : inPorts) {
        NetworkLink *inNetLink = iPort->inNetLink();
        if (inNetLink->isReady(curTick())) {
            scheduleEvent(Cycles(1));
            return;
        }
    }

    for (auto &oPort : outPorts) {
        CreditLink *inCreditLink = oPort->inCreditLink();
        if (inCreditLink->isReady(curTick())) {
            scheduleEvent(Cycles(1));
            return;
        }
    }
}

void
NetworkInterface::print(std::ostream& out) const
{
    out << "[Network Interface]";
}

bool
NetworkInterface::functionalRead(Packet *pkt, WriteMask &mask)
{
    bool read = false;
    for (auto& ni_out_vc : niOutVcs) {
        if (ni_out_vc.functionalRead(pkt, mask))
            read = true;
    }

    for (auto &oPort: outPorts) {
        if (oPort->outFlitQueue()->functionalRead(pkt, mask))
            read = true;
    }

    return read;
}

uint32_t
NetworkInterface::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (auto& ni_out_vc : niOutVcs) {
        num_functional_writes += ni_out_vc.functionalWrite(pkt);
    }

    for (auto &oPort: outPorts) {
        num_functional_writes += oPort->outFlitQueue()->functionalWrite(pkt);
    }
    return num_functional_writes;
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
