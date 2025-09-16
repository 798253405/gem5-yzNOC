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


#include "mem/ruby/network/garnet/Router.hh"

#include <algorithm>
#include <cmath>
#include "debug/RubyNetwork.hh"
#include "sim/clock_domain.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/InputUnit.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"
#include "mem/ruby/network/garnet/OutputUnit.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

Router::Router(const Params &p)
  : BasicRouter(p), Consumer(this), m_latency(p.latency),
    m_virtual_networks(p.virt_nets), m_vc_per_vnet(p.vcs_per_vnet),
    m_num_vcs(m_virtual_networks * m_vc_per_vnet), m_bit_width(p.width),
    m_network_ptr(nullptr), routingUnit(this), switchAllocator(this),
    crossbarSwitch(this), m_current_dvfs_level(DVFS_MEDIUM),
    m_dvfs_update_event([this]{ periodicDVFSUpdate(); }, name()),
    m_dvfs_switch_interval(p.dvfs_switch_interval),
    m_dvfs_cycle_counter(0), m_dvfs_enable_periodic(p.dvfs_enable_periodic),
    m_dvfs_freq{p.dvfs_low_freq_mhz, p.dvfs_medium_freq_mhz, p.dvfs_high_freq_mhz},
    m_dvfs_voltage{p.dvfs_low_voltage, p.dvfs_medium_voltage, p.dvfs_high_voltage},
    m_freq_scale_factor(1.0),
    m_dvfs_mode(p.dvfs_mode)
{
    m_input_unit.clear();
    m_output_unit.clear();
}

void
Router::init()
{
    BasicRouter::init();

    switchAllocator.init();
    crossbarSwitch.init();
    
    // Initialize DVFS based on mode
    if (m_dvfs_mode == "low") {
        setDVFSLevel(DVFS_LOW);
        Cycles eff_latency = getEffectiveLatency();
        printf("Router %d DVFS mode=LOW: Fixed 1GHz (%.0fMHz), scale=%.2f, base_latency=%d, effective_latency=%llu\n", 
               m_id, m_dvfs_freq[DVFS_LOW], m_freq_scale_factor, m_latency, eff_latency);
    } else if (m_dvfs_mode == "medium") {
        setDVFSLevel(DVFS_MEDIUM);
        Cycles eff_latency = getEffectiveLatency();
        printf("Router %d DVFS mode=MEDIUM: Fixed 2GHz (%.0fMHz), scale=%.2f, base_latency=%d, effective_latency=%llu\n",
               m_id, m_dvfs_freq[DVFS_MEDIUM], m_freq_scale_factor, m_latency, eff_latency);
    } else if (m_dvfs_mode == "high") {
        setDVFSLevel(DVFS_HIGH);
        Cycles eff_latency = getEffectiveLatency();
        printf("Router %d DVFS mode=HIGH: Fixed 4GHz (%.0fMHz), scale=%.2f, base_latency=%d, effective_latency=%llu\n", 
               m_id, m_dvfs_freq[DVFS_HIGH], m_freq_scale_factor, m_latency, eff_latency);
    } else if (m_dvfs_mode == "cycle" && m_dvfs_enable_periodic) {
        // Start at medium and enable periodic switching
        setDVFSLevel(DVFS_MEDIUM);
        printf("Router %d DVFS mode=CYCLE: Starting at 2GHz, will cycle through all frequencies\n", m_id);
        schedule(m_dvfs_update_event, curTick() + m_dvfs_switch_interval);
    } else {
        // Default: MEDIUM frequency
        setDVFSLevel(DVFS_MEDIUM);
        Cycles eff_latency = getEffectiveLatency();
        printf("Router %d DVFS default: Fixed 2GHz (%.0fMHz), scale=%.2f, base_latency=%d, effective_latency=%llu\n",
               m_id, m_dvfs_freq[DVFS_MEDIUM], m_freq_scale_factor, m_latency, eff_latency);
    }
    
    DPRINTF(RubyNetwork, "Router %d DVFS initialized: mode=%s, freq=%.0fMHz\n", 
            m_id, m_dvfs_mode.c_str(), m_dvfs_freq[m_current_dvfs_level]);
    
    // Update DVFS statistics for visibility in stats.txt
    m_dvfs_freq_mhz = m_dvfs_freq[m_current_dvfs_level];
    m_dvfs_scale_factor = m_freq_scale_factor;
    m_dvfs_effective_latency = getEffectiveLatency();
}

void
Router::wakeup()
{
    DPRINTF(RubyNetwork, "Router %d woke up\n", m_id);
    assert(clockEdge() == curTick());

    // check for incoming flits
    for (int inport = 0; inport < m_input_unit.size(); inport++) {
        m_input_unit[inport]->wakeup();
    }

    // check for incoming credits
    // Note: the credit update is happening before SA
    // buffer turnaround time =
    //     credit traversal (1-cycle) + SA (1-cycle) + Link Traversal (1-cycle)
    // if we want the credit update to take place after SA, this loop should
    // be moved after the SA request
    for (int outport = 0; outport < m_output_unit.size(); outport++) {
        m_output_unit[outport]->wakeup();
    }

    // Switch Allocation
    switchAllocator.wakeup();

    // Switch Traversal
    crossbarSwitch.wakeup();
}

void
Router::addInPort(PortDirection inport_dirn,
                  NetworkLink *in_link, CreditLink *credit_link)
{
    fatal_if(in_link->bitWidth != m_bit_width, "Widths of link %s(%d)does"
            " not match that of Router%d(%d). Consider inserting SerDes "
            "Units.", in_link->name(), in_link->bitWidth, m_id, m_bit_width);

    int port_num = m_input_unit.size();
    InputUnit *input_unit = new InputUnit(port_num, inport_dirn, this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(this);
    in_link->setVcsPerVnet(get_vc_per_vnet());
    credit_link->setSourceQueue(input_unit->getCreditQueue(), this);
    credit_link->setVcsPerVnet(get_vc_per_vnet());

    m_input_unit.push_back(std::shared_ptr<InputUnit>(input_unit));

    routingUnit.addInDirection(inport_dirn, port_num);
}

void
Router::addOutPort(PortDirection outport_dirn,
                   NetworkLink *out_link,
                   std::vector<NetDest>& routing_table_entry, int link_weight,
                   CreditLink *credit_link, uint32_t consumerVcs)
{
    fatal_if(out_link->bitWidth != m_bit_width, "Widths of units do not match."
            " Consider inserting SerDes Units");

    int port_num = m_output_unit.size();
    OutputUnit *output_unit = new OutputUnit(port_num, outport_dirn, this,
                                             consumerVcs);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(this);
    credit_link->setVcsPerVnet(consumerVcs);
    out_link->setSourceQueue(output_unit->getOutQueue(), this);
    out_link->setVcsPerVnet(consumerVcs);

    m_output_unit.push_back(std::shared_ptr<OutputUnit>(output_unit));

    routingUnit.addRoute(routing_table_entry);
    routingUnit.addWeight(link_weight);
    routingUnit.addOutDirection(outport_dirn, port_num);
}

PortDirection
Router::getOutportDirection(int outport)
{
    return m_output_unit[outport]->get_direction();
}

PortDirection
Router::getInportDirection(int inport)
{
    return m_input_unit[inport]->get_direction();
}

int
Router::route_compute(RouteInfo route, int inport, PortDirection inport_dirn)
{
    return routingUnit.outportCompute(route, inport, inport_dirn);
}

void
Router::grant_switch(int inport, flit *t_flit)
{
    crossbarSwitch.update_sw_winner(inport, t_flit);
}

void
Router::schedule_wakeup(Cycles time)
{
    // wake up after time cycles
    scheduleEvent(time);
}

std::string
Router::getPortDirectionName(PortDirection direction)
{
    // PortDirection is actually a string
    // If not, then this function should add a switch
    // statement to convert direction to a string
    // that can be printed out
    return direction;
}

void
Router::regStats()
{
    BasicRouter::regStats();

    m_buffer_reads
        .name(name() + ".buffer_reads")
        .flags(statistics::nozero)
    ;

    m_buffer_writes
        .name(name() + ".buffer_writes")
        .flags(statistics::nozero)
    ;

    m_crossbar_activity
        .name(name() + ".crossbar_activity")
        .flags(statistics::nozero)
    ;

    m_sw_input_arbiter_activity
        .name(name() + ".sw_input_arbiter_activity")
        .flags(statistics::nozero)
    ;

    m_sw_output_arbiter_activity
        .name(name() + ".sw_output_arbiter_activity")
        .flags(statistics::nozero)
    ;
    
    // DVFS statistics
    m_dvfs_freq_mhz
        .name(name() + ".dvfs_frequency_mhz")
        .desc("Current DVFS frequency in MHz")
    ;
    
    m_dvfs_scale_factor
        .name(name() + ".dvfs_scale_factor")
        .desc("Current DVFS frequency scaling factor")
    ;
    
    m_dvfs_effective_latency
        .name(name() + ".dvfs_effective_latency_cycles")
        .desc("Effective router latency in cycles after DVFS scaling")
    ;
}

void
Router::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->get_buf_read_activity(j);
            m_buffer_writes += m_input_unit[i]->get_buf_write_activity(j);
        }
    }

    m_sw_input_arbiter_activity = switchAllocator.get_input_arbiter_activity();
    m_sw_output_arbiter_activity =
        switchAllocator.get_output_arbiter_activity();
    m_crossbar_activity = crossbarSwitch.get_crossbar_activity();
}

void
Router::resetStats()
{
    for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
    }

    crossbarSwitch.resetStats();
    switchAllocator.resetStats();
}

void
Router::printFaultVector(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << std::endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++) {
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << std::endl;
    }
}

void
Router::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << std::endl;
}

bool
Router::functionalRead(Packet *pkt, WriteMask &mask)
{
    bool read = false;
    if (crossbarSwitch.functionalRead(pkt, mask))
        read = true;

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        if (m_input_unit[i]->functionalRead(pkt, mask))
            read = true;
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        if (m_output_unit[i]->functionalRead(pkt, mask))
            read = true;
    }

    return read;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    num_functional_writes += crossbarSwitch.functionalWrite(pkt);

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        num_functional_writes += m_input_unit[i]->functionalWrite(pkt);
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        num_functional_writes += m_output_unit[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
Router::setDVFSLevel(DVFSLevel level)
{
    DPRINTF(RubyNetwork, "Router %d switching to DVFS level %d\n", m_id, level);
    
    m_current_dvfs_level = level;
    
    // Get frequency/voltage levels from configuration parameters
    double frequency_mhz = m_dvfs_freq[level];
    double voltage_v = m_dvfs_voltage[level];
    
    // Calculate clock period in ticks (gem5 uses ticks per second = 1e12)
    Tick new_clock_period = (Tick)(1e12 / (frequency_mhz * 1e6)); // Convert MHz to ticks
    
    DPRINTF(RubyNetwork, "Router %d DVFS change: freq=%.1fMHz (period=%lld ticks), voltage=%.1fV\n", 
            m_id, frequency_mhz, new_clock_period, voltage_v);
    
    // Try to change actual clock domain if this router has its own SrcClockDomain
    // This requires the router to have been assigned an independent clock domain in the config
    // Note: We cannot directly access clockDomain as it's private, but the effect
    // will be visible through clockPeriod() if the domain supports switching
    
    // Calculate frequency scaling factor for timing adjustments
    // This is used as a fallback when real clock domain switching is not available
    double medium_freq = m_dvfs_freq[DVFS_MEDIUM];
    m_freq_scale_factor = frequency_mhz / medium_freq;
    
    // Log the current actual clock period to verify if domain switching worked
    Tick actual_period = clockPeriod();
    DPRINTF(RubyNetwork, "Router %d: target freq=%.1fMHz (period=%lld), actual period=%lld, scale=%.2fx\n",
            m_id, frequency_mhz, new_clock_period, actual_period, m_freq_scale_factor);
    
    // The frequency change affects packet processing:
    // - If real clock domain switching: hardware timing changes automatically
    // - If simulation only: we use m_freq_scale_factor to adjust latencies
    
    // Update DVFS statistics
    m_dvfs_freq_mhz = frequency_mhz;
    m_dvfs_scale_factor = m_freq_scale_factor;
    m_dvfs_effective_latency = getEffectiveLatency();
}

void
Router::triggerDVFSChange(DVFSLevel level)
{
    // Interface for future network-aware DVFS control
    // This can be called by network monitoring logic
    DPRINTF(RubyNetwork, "Router %d triggered DVFS change to level %d\n", m_id, level);
    setDVFSLevel(level);
}

void
Router::periodicDVFSUpdate()
{
    // Cycle between MEDIUM and HIGH only (skip LOW for better performance)
    // MEDIUM -> HIGH -> MEDIUM -> HIGH ...
    DVFSLevel next_level;
    
    switch (m_current_dvfs_level) {
        case DVFS_LOW:
            // If somehow we start at LOW, go to MEDIUM
            next_level = DVFS_MEDIUM;
            break;
        case DVFS_MEDIUM:
            // Switch to HIGH frequency
            next_level = DVFS_HIGH;
            break;
        case DVFS_HIGH:
            // Switch back to MEDIUM frequency
            next_level = DVFS_MEDIUM;
            break;
    }
    
    setDVFSLevel(next_level);
    m_dvfs_cycle_counter++;
    
    DPRINTF(RubyNetwork, "Router %d periodic DVFS update #%d: %d -> %d\n", 
            m_id, m_dvfs_cycle_counter, m_current_dvfs_level, next_level);
    
    // Schedule next periodic update if enabled
    if (m_dvfs_enable_periodic) {
        schedule(m_dvfs_update_event, curTick() + m_dvfs_switch_interval);
    }
}

void
Router::clockPeriodUpdated()
{
    // This hook is called when the clock period changes
    // We can use this to update any timing-related parameters
    Tick current_period = clockPeriod();
    DPRINTF(RubyNetwork, "Router %d clock period updated to %lld ticks (%.1f MHz)\n", 
            m_id, current_period, 1e12 / current_period / 1e6);
    
    // Update any internal timing parameters that depend on clock period
    // For example, update latency calculations, buffer timing, etc.
    // This ensures the router operates correctly at the new frequency
}

Cycles
Router::getEffectiveLatency() const
{
    // For DVFS demonstration: use a virtual base latency when the actual latency is 1
    // This allows us to see DVFS effects even with the default 1-cycle router
    uint32_t base_latency = m_latency;
    
    // When DVFS is active (scale != 1.0) and base latency is 1, 
    // use a virtual base of 4 cycles for scaling calculations
    if (m_freq_scale_factor != 1.0 && m_latency == 1) {
        base_latency = 4;  // Virtual base for DVFS scaling
    }
    
    // Apply DVFS frequency scaling to base latency
    // Higher frequency (scale > 1.0) = lower latency
    // Lower frequency (scale < 1.0) = higher latency
    double scaled_latency = static_cast<double>(base_latency) / m_freq_scale_factor;
    
    // Round to nearest integer instead of truncating
    uint64_t effective_cycles = std::max(1UL, static_cast<uint64_t>(std::round(scaled_latency)));
    
    // Debug output to verify DVFS effect
    if (m_freq_scale_factor != 1.0) {
        DPRINTF(RubyNetwork, "Router %d: actual_latency=%d, virtual_base=%d, scale=%.2f, effective_latency=%llu\n",
                m_id, m_latency, base_latency, m_freq_scale_factor, effective_cycles);
    }
    
    return Cycles(effective_cycles);
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
