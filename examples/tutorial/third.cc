/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#undef PGO_TRAINING
#define PATH_TO_PGO_CONFIG "path_to_pgo_config"

#include <iostream>
#include <fstream>
#include <time.h> 
#include <assert.h>
#include "ns3/core-module.h"
#include "ns3/qbb-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/global-route-manager.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/broadcom-node.h"
#include "ns3/packet.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-probe.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/udp-client-server-helper.h"



using namespace ns3;

class my_app_data {
public:
	my_app_data(uint32_t port, uint32_t maxpacketcount, Ptr<Application> client, Ptr<Application> server);
	uint32_t port;
	uint32_t maxpacketcount;
	Ptr<Application> client;
	Ptr<Application> server;
	bool handled;
};
my_app_data::my_app_data(uint32_t port, uint32_t maxpacketcount, Ptr<Application> client, Ptr<Application> server) {
	this->port = port;
	this->maxpacketcount = maxpacketcount;
	this->client = client;
	this->server = server;
	this->handled = false;
}

std::vector< my_app_data* > app_data_container;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

bool enable_qcn = true, use_dynamic_pfc_threshold = true, packet_level_ecmp = false, flow_level_ecmp = false;
uint32_t packet_payload_size = 1000, l2_chunk_size = 0, l2_ack_interval = 0;
double pause_time = 5, simulator_stop_time = 3.01, app_start_time = 1.0, app_stop_time = 9.0;
std::string data_rate, link_delay, topology_file, flow_file, tcp_flow_file, trace_file, trace_output_file, xml_output_file;
bool used_port[65536] = { 0 };

double cnp_interval = 50, alpha_resume_interval = 55, rp_timer, dctcp_gain = 1 / 16, np_sampling_interval = 0, pmax = 1;
uint32_t byte_counter, fast_recovery_times = 5, kmax = 60, kmin = 60, buffer_size = 1000000;
std::string rate_ai, rate_hai;

bool clamp_target_rate = false, clamp_target_rate_after_timer = false, send_in_chunks = true, l2_wait_for_ack = false, l2_back_to_zero = false, l2_test_read = false;
double error_rate_per_link = 0.0;

uint32_t packetSize, pg;
Time interPacketInterval;

void handler_print_time();
void handler_create_app(uint32_t src, uint32_t dst, uint32_t maxPacketCount, uint32_t pg);
NodeContainer n;
//ApplicationContainer app_container_server;
//ApplicationContainer app_container_client;

uint32_t app_counter = 0;
uint32_t stopped_app_counter = 0;

void handler_print_time() {
	Time current_time = Simulator::Now();
	double current_time_seconds = current_time.GetSeconds();
	std::cout << "Current simulation time: " << current_time_seconds << " seconds" << std::endl;
	Simulator::Schedule(Seconds(0.005), &handler_print_time);
}

void handler_create_app(uint32_t src, uint32_t dst, uint32_t maxPacketCount, uint32_t pg) {
	// for all existing applications, check whether the corresponding server has received the maxPacketCount
	// if yes, stop the client-server, free the port

	//update the port ...
	//std::cout << "here 1\n";
	for (int i = app_data_container.size() - 1; i >= 0; i--) {
		//std::cout << i <<  " here 2\n";
		my_app_data *app_data = app_data_container[i];
		//std::cout << i << " here 3 " << app_data->handled << "\n";
		if (app_data->handled == true) {
			continue;
		}
		//std::cout << "here 3\n";
		Ptr<Application> client_app = app_data->client;
		Ptr<Application> server_app = app_data->server;
		uint32_t port_used = app_data->port;
		uint32_t maxpacketcount = app_data->maxpacketcount;

		// get the maxpacketcount from client and the received count from server, get the port from server
		// if both are same of received >= maxpacketcount, free the port, stop the client and server
		uint32_t packets_received;
		if (send_in_chunks) {
			assert(false);
		}
		else {
			packets_received = DynamicCast<UdpServer>(server_app)->GetReceived();
		}

		if (packets_received >= maxpacketcount) {
			used_port[port_used] = false;
			//std::cout << "Port " << port_used << " freed" << std::endl;
			app_data->handled = true;
			client_app->SetStopTime(Seconds(0));
			server_app->SetStopTime(Seconds(0));
			stopped_app_counter++;
			if (stopped_app_counter % 1000 == 0) {
				std::cout << "Stopped apps: " << stopped_app_counter << std::endl;
			}
			app_data_container.erase(app_data_container.begin() + i);
			free(app_data);
		}
	}

	// create the new client-server here
	uint32_t port = int(UniformVariable(0, 1).GetValue() * 40000);
	while (used_port[port]) {
		port = int(UniformVariable(0, 1).GetValue() * 40000);
		continue;
	}
	used_port[port] = true;
	NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
	Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
	Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1
	if (send_in_chunks)
	{
		assert(false);
	}
	else
	{
		//std::cout << "Port used: " << port << std::endl;
		UdpServerHelper server0(port);
		ApplicationContainer app0s_c = server0.Install(n.Get(dst));
		Ptr<Application> app0s = app0s_c.Get(0);
		app0s->SetStartTime(Seconds(0));
		app0s->SetStopTime(Seconds(10));
		//app_container_server.Add(app0s);

		UdpClientHelper client0(serverAddress, port, pg); //Add Priority
		client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
		client0.SetAttribute("Interval", TimeValue(interPacketInterval));
		client0.SetAttribute("PacketSize", UintegerValue(packetSize));
		ApplicationContainer app0c_c = client0.Install(n.Get(src));
		Ptr<Application> app0c = app0c_c.Get(0);
		app0c->SetStartTime(Seconds(0));
		app0c->SetStopTime(Seconds(10));
		//app_container_client.Add(app0c);

		my_app_data *app_data_ptr = new my_app_data(port, maxPacketCount, app0c, app0s);
		app_data_container.push_back(app_data_ptr);
		app_counter++;
		if (app_counter % 1000 == 0) {
			std::cout << " App " << app_counter << " created" << std::endl;
		}
	}


}

int main(int argc, char *argv[])
{
	clock_t begint, endt;
	begint = clock();
#ifndef PGO_TRAINING
	if (argc > 1)
#else
	if (true)
#endif
	{
		//Read the configuration file
		std::ifstream conf;
#ifndef PGO_TRAINING
		conf.open(argv[1]);
#else
		conf.open(PATH_TO_PGO_CONFIG);
#endif
		while (!conf.eof())
		{
			std::string key;
			conf >> key;

			//std::cout << conf.cur << "\n";

			if (key.compare("ENABLE_QCN") == 0)
			{
				uint32_t v;
				conf >> v;
				enable_qcn = v;
				if (enable_qcn)
					std::cout << "ENABLE_QCN\t\t\t" << "Yes" << "\n";
				else
					std::cout << "ENABLE_QCN\t\t\t" << "No" << "\n";
			}
			else if (key.compare("USE_DYNAMIC_PFC_THRESHOLD") == 0)
			{
				uint32_t v;
				conf >> v;
				use_dynamic_pfc_threshold = v;
				if (use_dynamic_pfc_threshold)
					std::cout << "USE_DYNAMIC_PFC_THRESHOLD\t" << "Yes" << "\n";
				else
					std::cout << "USE_DYNAMIC_PFC_THRESHOLD\t" << "No" << "\n";
			}
			else if (key.compare("CLAMP_TARGET_RATE") == 0)
			{
				uint32_t v;
				conf >> v;
				clamp_target_rate = v;
				if (clamp_target_rate)
					std::cout << "CLAMP_TARGET_RATE\t\t" << "Yes" << "\n";
				else
					std::cout << "CLAMP_TARGET_RATE\t\t" << "No" << "\n";
			}
			else if (key.compare("CLAMP_TARGET_RATE_AFTER_TIMER") == 0)
			{
				uint32_t v;
				conf >> v;
				clamp_target_rate_after_timer = v;
				if (clamp_target_rate_after_timer)
					std::cout << "CLAMP_TARGET_RATE_AFTER_TIMER\t" << "Yes" << "\n";
				else
					std::cout << "CLAMP_TARGET_RATE_AFTER_TIMER\t" << "No" << "\n";
			}
			else if (key.compare("PACKET_LEVEL_ECMP") == 0)
			{
				uint32_t v;
				conf >> v;
				packet_level_ecmp = v;
				if (packet_level_ecmp)
					std::cout << "PACKET_LEVEL_ECMP\t\t" << "Yes" << "\n";
				else
					std::cout << "PACKET_LEVEL_ECMP\t\t" << "No" << "\n";
			}
			else if (key.compare("FLOW_LEVEL_ECMP") == 0)
			{
				uint32_t v;
				conf >> v;
				flow_level_ecmp = v;
				if (flow_level_ecmp)
					std::cout << "FLOW_LEVEL_ECMP\t\t\t" << "Yes" << "\n";
				else
					std::cout << "FLOW_LEVEL_ECMP\t\t\t" << "No" << "\n";
			}
			else if (key.compare("PAUSE_TIME") == 0)
			{
				double v;
				conf >> v;
				pause_time = v;
				std::cout << "PAUSE_TIME\t\t\t" << pause_time << "\n";
			}
			else if (key.compare("DATA_RATE") == 0)
			{
				std::string v;
				conf >> v;
				data_rate = v;
				std::cout << "DATA_RATE\t\t\t" << data_rate << "\n";
			}
			else if (key.compare("LINK_DELAY") == 0)
			{
				std::string v;
				conf >> v;
				link_delay = v;
				std::cout << "LINK_DELAY\t\t\t" << link_delay << "\n";
			}
			else if (key.compare("PACKET_PAYLOAD_SIZE") == 0)
			{
				uint32_t v;
				conf >> v;
				packet_payload_size = v;
				std::cout << "PACKET_PAYLOAD_SIZE\t\t" << packet_payload_size << "\n";
			}
			else if (key.compare("L2_CHUNK_SIZE") == 0)
			{
				uint32_t v;
				conf >> v;
				l2_chunk_size = v;
				std::cout << "L2_CHUNK_SIZE\t\t\t" << l2_chunk_size << "\n";
			}
			else if (key.compare("L2_ACK_INTERVAL") == 0)
			{
				uint32_t v;
				conf >> v;
				l2_ack_interval = v;
				std::cout << "L2_ACK_INTERVAL\t\t\t" << l2_ack_interval << "\n";
			}
			else if (key.compare("L2_WAIT_FOR_ACK") == 0)
			{
				uint32_t v;
				conf >> v;
				l2_wait_for_ack = v;
				if (l2_wait_for_ack)
					std::cout << "L2_WAIT_FOR_ACK\t\t\t" << "Yes" << "\n";
				else
					std::cout << "L2_WAIT_FOR_ACK\t\t\t" << "No" << "\n";
			}
			else if (key.compare("L2_BACK_TO_ZERO") == 0)
			{
				uint32_t v;
				conf >> v;
				l2_back_to_zero = v;
				if (l2_back_to_zero)
					std::cout << "L2_BACK_TO_ZERO\t\t\t" << "Yes" << "\n";
				else
					std::cout << "L2_BACK_TO_ZERO\t\t\t" << "No" << "\n";
			}
			else if (key.compare("L2_TEST_READ") == 0)
			{
				uint32_t v;
				conf >> v;
				l2_test_read = v;
				if (l2_test_read)
					std::cout << "L2_TEST_READ\t\t\t" << "Yes" << "\n";
				else
					std::cout << "L2_TEST_READ\t\t\t" << "No" << "\n";
			}
			else if (key.compare("TOPOLOGY_FILE") == 0)
			{
				std::string v;
				conf >> v;
				topology_file = v;
				std::cout << "TOPOLOGY_FILE\t\t\t" << topology_file << "\n";
			}
			else if (key.compare("FLOW_FILE") == 0)
			{
				std::string v;
				conf >> v;
				flow_file = v;
				std::cout << "FLOW_FILE\t\t\t" << flow_file << "\n";
			}
			else if (key.compare("TCP_FLOW_FILE") == 0)
			{
				std::string v;
				conf >> v;
				tcp_flow_file = v;
				std::cout << "TCP_FLOW_FILE\t\t\t" << tcp_flow_file << "\n";
			}
			else if (key.compare("TRACE_FILE") == 0)
			{
				std::string v;
				conf >> v;
				trace_file = v;
				std::cout << "TRACE_FILE\t\t\t" << trace_file << "\n";
			}
			else if (key.compare("TRACE_OUTPUT_FILE") == 0)
			{
				std::string v;
				conf >> v;
				trace_output_file = v;
				if (argc > 2)
				{
					trace_output_file = trace_output_file + std::string(argv[2]);
				}
				std::cout << "TRACE_OUTPUT_FILE\t\t" << trace_output_file << "\n";
			}
			else if (key.compare("APP_START_TIME") == 0)
			{
				double v;
				conf >> v;
				app_start_time = v;
				std::cout << "SINK_START_TIME\t\t\t" << app_start_time << "\n";
			}
			else if (key.compare("APP_STOP_TIME") == 0)
			{
				double v;
				conf >> v;
				app_stop_time = v;
				std::cout << "SINK_STOP_TIME\t\t\t" << app_stop_time << "\n";
			}
			else if (key.compare("SIMULATOR_STOP_TIME") == 0)
			{
				double v;
				conf >> v;
				simulator_stop_time = v;
				std::cout << "SIMULATOR_STOP_TIME\t\t" << simulator_stop_time << "\n";
			}
			else if (key.compare("CNP_INTERVAL") == 0)
			{
				double v;
				conf >> v;
				cnp_interval = v;
				std::cout << "CNP_INTERVAL\t\t\t" << cnp_interval << "\n";
			}
			else if (key.compare("ALPHA_RESUME_INTERVAL") == 0)
			{
				double v;
				conf >> v;
				alpha_resume_interval = v;
				std::cout << "ALPHA_RESUME_INTERVAL\t\t" << alpha_resume_interval << "\n";
			}
			else if (key.compare("RP_TIMER") == 0)
			{
				double v;
				conf >> v;
				rp_timer = v;
				std::cout << "RP_TIMER\t\t\t" << rp_timer << "\n";
			}
			else if (key.compare("BYTE_COUNTER") == 0)
			{
				uint32_t v;
				conf >> v;
				byte_counter = v;
				std::cout << "BYTE_COUNTER\t\t\t" << byte_counter << "\n";
			}
			else if (key.compare("KMAX") == 0)
			{
				uint32_t v;
				conf >> v;
				kmax = v;
				std::cout << "KMAX\t\t\t\t" << kmax << "\n";
			}
			else if (key.compare("KMIN") == 0)
			{
				uint32_t v;
				conf >> v;
				kmin = v;
				std::cout << "KMIN\t\t\t\t" << kmin << "\n";
			}
			else if (key.compare("BUFFER_SIZE") == 0)
			{
				uint32_t v;
				conf >> v;
				buffer_size = v;
				std::cout << "BUFFER_SIZE\t\t\t\t" << buffer_size << "\n";
			}
			else if (key.compare("PMAX") == 0)
			{
				double v;
				conf >> v;
				pmax = v;
				std::cout << "PMAX\t\t\t\t" << pmax << "\n";
			}
			else if (key.compare("DCTCP_GAIN") == 0)
			{
				double v;
				conf >> v;
				dctcp_gain = v;
				std::cout << "DCTCP_GAIN\t\t\t" << dctcp_gain << "\n";
			}
			else if (key.compare("FAST_RECOVERY_TIMES") == 0)
			{
				uint32_t v;
				conf >> v;
				fast_recovery_times = v;
				std::cout << "FAST_RECOVERY_TIMES\t\t" << fast_recovery_times << "\n";
			}
			else if (key.compare("RATE_AI") == 0)
			{
				std::string v;
				conf >> v;
				rate_ai = v;
				std::cout << "RATE_AI\t\t\t\t" << rate_ai << "\n";
			}
			else if (key.compare("RATE_HAI") == 0)
			{
				std::string v;
				conf >> v;
				rate_hai = v;
				std::cout << "RATE_HAI\t\t\t" << rate_hai << "\n";
			}
			else if (key.compare("NP_SAMPLING_INTERVAL") == 0)
			{
				double v;
				conf >> v;
				np_sampling_interval = v;
				std::cout << "NP_SAMPLING_INTERVAL\t\t" << np_sampling_interval << "\n";
			}
			else if (key.compare("SEND_IN_CHUNKS") == 0)
			{
				uint32_t v;
				conf >> v;
				send_in_chunks = v;
				if (send_in_chunks)
				{
					std::cout << "SEND_IN_CHUNKS\t\t\t" << "Yes" << "\n";
					std::cout << "WARNING: deprecated and not tested. Please consider using L2_WAIT_FOR_ACK";
				}
				else
					std::cout << "SEND_IN_CHUNKS\t\t\t" << "No" << "\n";
			}
			else if (key.compare("ERROR_RATE_PER_LINK") == 0)
			{
				double v;
				conf >> v;
				error_rate_per_link = v;
				std::cout << "ERROR_RATE_PER_LINK\t\t" << error_rate_per_link << "\n";
			}
			else if (key.compare("XML_FILE") == 0)
			{
				std::string v;
				conf >> v;
				xml_output_file = v;
				if (argc > 3)
				{
					xml_output_file = xml_output_file + std::string(argv[3]);
				}
				std::cout << "XML_FILE\t\t" << xml_output_file << "\n";
			}
			fflush(stdout);
		}
		conf.close();
	}
	else
	{
		std::cout << "Error: require a config file\n";
		fflush(stdout);
		return 1;
	}


	bool dynamicth = use_dynamic_pfc_threshold;

	NS_ASSERT(packet_level_ecmp + flow_level_ecmp < 2); //packet level ecmp and flow level ecmp are exclusive
	Config::SetDefault("ns3::Ipv4GlobalRouting::RandomEcmpRouting", BooleanValue(packet_level_ecmp));
	Config::SetDefault("ns3::Ipv4GlobalRouting::FlowEcmpRouting", BooleanValue(flow_level_ecmp));
	Config::SetDefault("ns3::QbbNetDevice::PauseTime", UintegerValue(pause_time));
	Config::SetDefault("ns3::QbbNetDevice::QcnEnabled", BooleanValue(enable_qcn));
	Config::SetDefault("ns3::QbbNetDevice::DynamicThreshold", BooleanValue(dynamicth));
	Config::SetDefault("ns3::QbbNetDevice::ClampTargetRate", BooleanValue(clamp_target_rate));
	Config::SetDefault("ns3::QbbNetDevice::ClampTargetRateAfterTimeInc", BooleanValue(clamp_target_rate_after_timer));
	Config::SetDefault("ns3::QbbNetDevice::CNPInterval", DoubleValue(cnp_interval));
	Config::SetDefault("ns3::QbbNetDevice::NPSamplingInterval", DoubleValue(np_sampling_interval));
	Config::SetDefault("ns3::QbbNetDevice::AlphaResumInterval", DoubleValue(alpha_resume_interval));
	Config::SetDefault("ns3::QbbNetDevice::RPTimer", DoubleValue(rp_timer));
	Config::SetDefault("ns3::QbbNetDevice::ByteCounter", UintegerValue(byte_counter));
	Config::SetDefault("ns3::QbbNetDevice::FastRecoveryTimes", UintegerValue(fast_recovery_times));
	Config::SetDefault("ns3::QbbNetDevice::DCTCPGain", DoubleValue(dctcp_gain));
	Config::SetDefault("ns3::QbbNetDevice::RateAI", DataRateValue(DataRate(rate_ai)));
	Config::SetDefault("ns3::QbbNetDevice::RateHAI", DataRateValue(DataRate(rate_hai)));
	Config::SetDefault("ns3::QbbNetDevice::L2BackToZero", BooleanValue(l2_back_to_zero));
	Config::SetDefault("ns3::QbbNetDevice::L2TestRead", BooleanValue(l2_test_read));
	Config::SetDefault("ns3::QbbNetDevice::L2ChunkSize", UintegerValue(l2_chunk_size));
	Config::SetDefault("ns3::QbbNetDevice::L2AckInterval", UintegerValue(l2_ack_interval));
	Config::SetDefault("ns3::QbbNetDevice::L2WaitForAck", BooleanValue(l2_wait_for_ack));

	SeedManager::SetSeed(time(NULL));

	std::ifstream topof, flowf, tracef, tcpflowf;
	topof.open(topology_file.c_str());
	flowf.open(flow_file.c_str());
	tracef.open(trace_file.c_str());
	tcpflowf.open(tcp_flow_file.c_str());
	uint32_t node_num, switch_num, link_num, flow_num, trace_num, tcp_flow_num;
	topof >> node_num >> switch_num >> link_num;
	flowf >> flow_num;
	tracef >> trace_num;
	tcpflowf >> tcp_flow_num;



	n.Create(node_num);
	for (uint32_t i = 0; i < switch_num; i++)
	{
		uint32_t sid;
		topof >> sid;
		n.Get(sid)->SetNodeType(1, dynamicth); //broadcom switch
		n.Get(sid)->m_broadcom->SetMarkingThreshold(kmin, kmax, pmax);
	}


	NS_LOG_INFO("Create nodes.");

	InternetStackHelper internet;
	internet.Install(n);

	NS_LOG_INFO("Create channels.");

	//
	// Explicitly create the channels required by the topology.
	//

	Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
	Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
	rem->SetRandomVariable(uv);
	uv->SetStream(50);
	rem->SetAttribute("ErrorRate", DoubleValue(error_rate_per_link));
	rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));

	QbbHelper qbb;
	Ipv4AddressHelper ipv4;
	for (uint32_t i = 0; i < link_num; i++)
	{
		uint32_t src, dst;
		std::string data_rate, link_delay;
		double error_rate;
		topof >> src >> dst >> data_rate >> link_delay >> error_rate;

		qbb.SetDeviceAttribute("DataRate", StringValue(data_rate));
		qbb.SetChannelAttribute("Delay", StringValue(link_delay));

		if (error_rate > 0)
		{
			Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
			Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
			rem->SetRandomVariable(uv);
			uv->SetStream(50);
			rem->SetAttribute("ErrorRate", DoubleValue(error_rate));
			rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
			qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
		}
		else
		{
			qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
		}

		fflush(stdout);
		NetDeviceContainer d = qbb.Install(n.Get(src), n.Get(dst));

		char ipstring[16];
		sprintf(ipstring, "10.%d.%d.0", i / 254 + 1, i % 254 + 1);
		ipv4.SetBase(ipstring, "255.255.255.0");
		ipv4.Assign(d);
		std::cout << "Parsed link " << i << std::endl;
	}


	NodeContainer trace_nodes;
	for (uint32_t i = 0; i < trace_num; i++)
	{
		uint32_t nid;
		tracef >> nid;
		trace_nodes = NodeContainer(trace_nodes, n.Get(nid));
	}
	AsciiTraceHelper ascii;
	qbb.EnableAscii(ascii.CreateFileStream(trace_output_file), trace_nodes);

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	NS_LOG_INFO("Create Applications.");

	packetSize = packet_payload_size;
	interPacketInterval = Seconds(0.0000005 / 2);

	// Start and stop the required applications depending upon the flow trace now
	// Once an application is stopped, the port occupied by the client-server is cleared

	for (uint32_t i = 0; i < flow_num; i++) {
		uint32_t src, dst, pg, maxPacketCount;
		double start_time, stop_time;
		flowf >> src >> dst >> pg >> maxPacketCount >> start_time >> stop_time;
		// stop_time is assumed to be a very large value, actual stop time of application is depended upon the maxPacketCount
		Simulator::Schedule(Seconds(start_time), &handler_create_app, src, dst, maxPacketCount, pg);
		if (i % 10000 == 0) {
			std::cout << "Flow " << i << " parsed" << std::endl;
		}
	}

	Simulator::Schedule(Seconds(2), &handler_print_time);

	//
	// Now, do the actual simulation.
	//

	Ptr<FlowMonitor> flowmonitor;
	FlowMonitorHelper flowmhelper;
	flowmonitor = flowmhelper.InstallAll();


	std::cout << "Running Simulation.\n";
	fflush(stdout);
	NS_LOG_INFO("Run Simulation.");
	Simulator::Stop(Seconds(simulator_stop_time));
	Simulator::Run();




	//for (uint32_t i = 0; i < tcp_flow_num; i++)
	//{
	//	uint32_t src, dst, pg, maxPacketCount, port;
	//	double start_time, stop_time;
	//	while (used_port[port = int(UniformVariable(0, 1).GetValue() * 40000)])
	//		continue;
	//	used_port[port] = true;
	//	tcpflowf >> src >> dst >> pg >> maxPacketCount >> start_time >> stop_time;
	//	NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
	//	Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
	//	Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal();

	//	Address sinkLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
	//	PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkLocalAddress);

	//	ApplicationContainer sinkApp = sinkHelper.Install(n.Get(dst));
	//	sinkApp.Start(Seconds(app_start_time));
	//	sinkApp.Stop(Seconds(app_stop_time));

	//	BulkSendHelper source("ns3::TcpSocketFactory", InetSocketAddress(serverAddress, port));
	//	// Set the amount of data to send in bytes.  Zero is unlimited.
	//	source.SetAttribute("MaxBytes", UintegerValue(0));
	//	ApplicationContainer sourceApps = source.Install(n.Get(src));
	//	sourceApps.Start(Seconds(start_time));
	//	sourceApps.Stop(Seconds(stop_time));
	//}


	topof.close();
	flowf.close();
	tracef.close();
	tcpflowf.close();

	////////////////////////////////////////////////////////////////////////////////////

	flowmonitor->SerializeToXmlFile(xml_output_file, true, true);
	Simulator::Destroy();
	NS_LOG_INFO("Done.");

	endt = clock();
	std::cout << (double)(endt - begint) / CLOCKS_PER_SEC << "\n";

}
