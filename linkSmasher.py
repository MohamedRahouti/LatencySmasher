#from astar import astar
import json
import requests
import os
from httplib import HTTPConnection
from os import _exit
from random import randint
from sys import exit
from traceback import print_exc
import re
import httplib
from array import *
import numpy as np
from time import time
from netifaces import interfaces
import netifaces as ni


class astar(object):

    def __init__(self, data):
        self.matrix = data
        self.g = 0
        self.h = 0

    def find_all_paths(self, graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return [path]
        if start not in graph:
            if not graph.has_key(start):
                return []
        paths = []
        for node in graph[start]:
            if node not in path:
                newpaths = self.find_all_paths(graph, node, end, path)
                for newpath in newpaths:
                    paths.append(newpath)
        return paths

    def fit_matrix(self, matrix, start, end, path=[]):
        path = path + [start]
        """Construct a graph (dictionary)first."""
        graph = {}
        g_matrix = matrix[0][2]
        for i in range(len(g_matrix)):
            j_index = 0
            if i == 0 or i == (len(g_matrix))-1:
                j_index = 1 
            cur_dpid = g_matrix[i][j_index][0] #was originally [i][0][0]
            #print("Current DPID in fit_matrix is... %s" %cur_dpid)
            if cur_dpid != 0:
                cur_path = []
                for j in range(len(g_matrix[i])):
                    """check by number of hops"""
                    if (g_matrix[i][j][13] != 0):
                        """add number of hops and port"""
                        cur_path.append(g_matrix[i][j][1])
                        cur_path.append(g_matrix[i][j][12])
                        cur_path.append(g_matrix[i][j][13])
                if len(cur_path) != 0:
                    graph[cur_dpid] = cur_path
        print("The resulting graph after transformation is...............")
        print(graph)
        return graph

    def find_hs(self, gr, paths, src, dst):
        graph_h = []
        for i in range(len(paths)):
            graph_h.append([])
            if len(paths[i]) == 2:
                graph_h[i].append(0)
            else:
                j = len(paths[i])-2
                graph_h[i].append(0)
                prev_h_indice = 0
                while j >= 0:
                    h = 0
                    h += int(self.find_g(gr, paths[i][j], paths[i][j+1])) + \
                                     int(graph_h[i][prev_h_indice])
                    graph_h[i].append(h)
                    prev_h_indice += 1
                    j -= 1
        return graph_h


    def find_g(self, g, node1, node2):
        list_by_key = g[node1]
        #print("node1 is ................")
        #print(node1)
        #print("list_by_key............")
        #print(list_by_key)
        gn = 0
        i = 0
        while i < (len(list_by_key)):
            if list_by_key[i] == node2:
                gn = list_by_key[i+2]
                #print ("gn is ............")
                #print(gn)
                break
            i += 2
        return gn


    def a_star(self, matrix, start_dpid, goal_dpid):
        if self.is_goal_reached(start_dpid, goal_dpid):
            print("Source and Destination are the same")
            return None
        gr_matrix = self.fit_matrix(matrix, start_dpid, goal_dpid)
        paths = self.find_all_paths(gr_matrix, start_dpid, goal_dpid)
        print("All paths between "+start_dpid+ " and "+goal_dpid+ " are:")
        print(paths)
        list_hs = self.find_hs(gr_matrix, paths, start_dpid, goal_dpid)
        best_fscore = []
        indice = 0
        for i in range(len(list_hs)):
            total_cost = 0
            for j in range(len(list_hs[i])):
                total_cost += list_hs[i][j]
            best_fscore.append(total_cost)
        min_fscore = best_fscore[0]
        print("The list of scores:")
        print(best_fscore)
        for i in range(len(best_fscore)):
            if min_fscore > best_fscore[i]:
                min_fscore = best_fscore[i]
                indice = i
        return paths[indice]


    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    """main function."""

    def random_graph(self):
        graph = {}
        for i in range(1000):
            node_id = i
            if node_id not in graph:
                graph[node_id] = ''
        for node_i in graph:
            edges = []
            for node_j in graph:
                if node_i != node_j:
                    edges.append(node_j)
                    edges.append(random.randint(0,9))
            graph[node_i] = edges    
	    
        return graph


class LinkSmasher(object):
    """Main LinkSmasher Class."""

    def __init__(self, __in_s_controller_ip, __in_i_controller_port):
        """Initializer."""
        self.s_controller_ip = __in_s_controller_ip
        self.i_controller_port = __in_i_controller_port

    def json_extractor(self, s_path):
        """Extract and return a list of dpids of all active ovss."""
        json_ovs_addr = self.json_rest_call('', 'GET', s_path)
        s_list_ovss_dpids = []
        if json_ovs_addr is not None:
            s_ovss_dpids = re.split(r'[,/"}{\s]\s*', json_ovs_addr)
            for i in range(len(s_ovss_dpids)):
                if len(s_ovss_dpids[i]) == 23 and re.match("[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]\:[A-Za-z0-9][A-Za-z0-9]", s_ovss_dpids[i]):
                    s_list_ovss_dpids.append(s_ovss_dpids[i])
            del s_ovss_dpids, json_ovs_addr 
            return s_list_ovss_dpids
        else:
            del json_ovs_addr
            return None

    def link_exist(self, s_dpid_src, s_dpid_dst):
        """link_exist function check whether or not 2 ovss are neighbors."""
        """Returns dpid_src, port_src, dpid_dst, port_dst, and latency."""
        s_external_path = '/wm/topology/external-links/json'
        if s_dpid_src == s_dpid_dst:
            return None
        a_extern_links = self.json_rest_call('', 'GET', s_external_path)
        if a_extern_links is None:
            print("Unable to collect link statistics, make sure stats enabled!")
            del a_extern_links
            return None
        else:
            s_extern_links = re.split(r'[,"}{\s]\s*', a_extern_links)
            """The json output shows src-switch:dpid. That's why I collect the
            information of curent cell+2. The following lists are to store the
            the information of all source and destination switches.
            """
            list_src_ovs = [] #list of all source switches
            list_dst_ovs = [] #list of all destination switches
            i_list_src_port = [] #list of ports of source OVSs
            i_list_dst_port = [] #list of ports of destination OVSs
            i_list_latency = [] #list of latencies between each pair of ovs
            """Check first which ovs is src and wich one is dst as the statistics for the             biderctional link may NOT be the same for both directions. Once this search is            done by using following key words, the stats information is located in a 
            couple cells away from the current indice."""
            for i, item in enumerate(s_extern_links):
                if item == "src-switch":
                    list_src_ovs.append(i+2)
                if item == "dst-switch":
                    list_dst_ovs.append(i+2)
                if item == "src-port":
                    i_list_src_port.append(i+1)
                if item == "dst-port":
                    i_list_dst_port.append(i+1)
                if item == "latency":
                    i_list_latency.append(i+1)
            s_list_src_dpids = []
            s_list_dst_dpids = []
            s_list_src_ports = []
            s_list_dst_ports = []
            s_list_latencies = []
            for i in range(len(list_src_ovs)):
                s_list_src_dpids.append(s_extern_links[list_src_ovs[i]])
                s_list_dst_dpids.append(s_extern_links[list_dst_ovs[i]])
                s_list_src_ports.append(s_extern_links[i_list_src_port[i]])
                s_list_dst_ports.append(s_extern_links[i_list_dst_port[i]])
                s_list_latencies.append(s_extern_links[i_list_latency[i]])
            src_dst_ports = [] 
            for i in range(len(s_list_src_dpids)):
                if (s_list_src_dpids[i] == s_dpid_src and s_list_dst_dpids[i] ==
                    s_dpid_dst):
                    src_dst_ports.append(s_dpid_src)
                    i_port_src = s_list_src_ports[i].split(':')
                    src_dst_ports.append(str(i_port_src[1]))
                    src_dst_ports.append(s_dpid_dst)
                    i_port_dst = s_list_dst_ports[i].split(':')
                    src_dst_ports.append(str(i_port_dst[1]))
                    latency = re.split(r':', s_list_latencies[i])
                    src_dst_ports.append(latency[1])
                if (s_list_src_dpids[i] == s_dpid_dst and s_list_dst_dpids[i] ==
                    s_dpid_src):
                    src_dst_ports.append(s_dpid_dst)
                    i_port_src = s_list_src_ports[i].split(':')
                    src_dst_ports.append(str(i_port_src[1]))
                    src_dst_ports.append(s_dpid_src)
                    i_port_dst = s_list_dst_ports[i].split(':')
                    src_dst_ports.append(str(i_port_dst[1]))
                    latency = re.split(r':', s_list_latencies[i])
                    src_dst_ports.append(latency[1])
            return src_dst_ports

    def link_statistic(self, s_src_dpid, s_src_port):
        """link_statistic returns a vector of statistics for a given link."""
        """The vector will contain [link_tx_speed, link_rx_speed, link_speed]."""
        s_vector_stats = []
        a_link_stats = self.json_rest_call('', 'GET',
                                              '/wm/statistics/bandwidth/' +
                                              str(s_src_dpid) + '/' +
                                              str(s_src_port) + '/json')
        if a_link_stats is None:
            print("Error with collectinf statistics, check statistics collector!")
            del a_link_stats
            return None
        else:
            s_stats = re.split(r'[,"}{\s]\s*', a_link_stats)
            s_list_ports = []
            s_list_rx_speed = []
            s_list_tx_speed = []
            s_list_link_speed = []
            for i in range(len(s_stats)):
                if s_stats[i] == "port":
                    s_list_ports.append(s_stats[i+2])
                if s_stats[i] == "bits-per-second-rx":
                    s_list_rx_speed.append(s_stats[i+2])
                if s_stats[i] == "bits-per-second-tx":
                    s_list_tx_speed.append(s_stats[i+2])
                if s_stats[i] == "link-speed-bits-per-second":
                    s_list_link_speed.append(s_stats[i+2])
            s_link_utilization_rx = s_list_rx_speed[s_list_ports.index(str(s_src_port))]
            s_link_utilization_tx = s_list_tx_speed[s_list_ports.index(str(s_src_port))]
            s_link_speed = s_list_link_speed[s_list_ports.index(str(s_src_port))]
            """isinstance to validate the corectness of the API output. If 
            the value is inconvertible to int, we cannot use it as a link 
            bandwidth value."""
            """Make sure that the link bandwidth/throughput are integers. 
            Otherwise rest_api does NOT return correct statistics"""
            if isinstance(int(s_link_utilization_rx), int) and isinstance(int(s_link_utilization_tx), int) and isinstance(int(s_link_speed), int):
                s_vector_stats. append(s_link_utilization_rx)
                s_vector_stats.append(s_link_utilization_tx)
                s_vector_stats.append(s_link_speed)
                del s_link_utilization_rx, s_link_utilization_tx, s_link_speed, s_stats, a_link_stats
                return s_vector_stats
            else:
                del s_link_speed, s_link_utilization, s_stats, json_link_stats
                return None


    def json_get(self):
        """Insert and return a topology data in matrix format.
        Ther are 3 major steps to establish the data matrix:
        step 1: insert/push the Dpids off all active ovss.
        step 2: check whether each pair of ovss are physicaly connected. 
        step3: if there is a physical link, push/insert its statistics to the matirx.
        1- dpid_src,2- dpid_dst,3-port_src,4-latency,5-rx_speed,6-tx_speed,7-speed,8-hop
        """
        """1. Call rest API to get DPIDs of ovs switches connected to SDN."""
        s_list_dpids = self.json_extractor('/wm/core/controller/switches/json')
        if s_list_dpids is None:
            print("There is a problem in json_extractor returned value")
            del s_list_dpids
            return None
        else:
            """Call rest API to get DPIDs of ovs switches connected to SDN."""
            i_nbr_ovss = len(s_list_dpids)
            s_get_matrix = [[[0 for i in range(14)] for j in range(i_nbr_ovss)]
                          for k in range(i_nbr_ovss)]
            """Call link_exist to test whether each pair of ovss are connected."""
            for i_dpid in range(i_nbr_ovss):
                for j_dpid in range(i_nbr_ovss):
                    if i_dpid != j_dpid:
                        s_get_matrix[i_dpid][j_dpid][0] = s_list_dpids[i_dpid]
                        s_get_matrix[i_dpid][j_dpid][1] = s_list_dpids[j_dpid]
                        """2. link_exist returns a vector of the link stats if there is a physical link betwenn 2 ovss."""
                        s_vector_lat_port = self.link_exist(s_get_matrix[i_dpid][j_dpid][0], s_get_matrix[i_dpid][j_dpid][1])
                        """The following blocks of statments cannot be shrinked. The first if statment to make sure the returned vector is NOT empty."""
                        if len(s_vector_lat_port) != 0:
                            """statment to test if a port is source or dest."""
                            i_port_src = s_vector_lat_port[s_vector_lat_port.index(str(s_get_matrix[i_dpid][j_dpid][0])) + 1]
                            s_get_matrix[i_dpid][j_dpid][2] = 'port'
                            s_get_matrix[i_dpid][j_dpid][3] = i_port_src
                            s_get_matrix[i_dpid][j_dpid][4] = 'lat'
                            s_get_matrix[i_dpid][j_dpid][5] = s_vector_lat_port[len(s_vector_lat_port)-1]
                            s_link_stats = self.link_statistic(s_get_matrix[i_dpid][j_dpid][0], s_get_matrix[i_dpid][j_dpid][3])
                            s_get_matrix[i_dpid][j_dpid][6] = 'rx_speed'
                            s_get_matrix[i_dpid][j_dpid][7] = s_link_stats[0]
                            s_get_matrix[i_dpid][j_dpid][8] = 'tx_speed'
                            s_get_matrix[i_dpid][j_dpid][9] = s_link_stats[1]
                            s_get_matrix[i_dpid][j_dpid][10] = 'link_speed'
                            s_get_matrix[i_dpid][j_dpid][11] = s_link_stats[2]
                            s_get_matrix[i_dpid][j_dpid][12] = 'hops'
                            s_get_matrix[i_dpid][j_dpid][13] = '1'
            SDN_addr = ni.ifaddresses('eth0')[2][0]['addr']
            st_time_stamp = time()
            s_matrix = np.array([[st_time_stamp, SDN_addr, s_get_matrix],
                               [st_time_stamp, SDN_addr, [6, 7, 8]], []])
            del st_time_stamp, SDN_addr, s_link_stats, s_vector_lat_port, s_get_matrix, s_list_dpids
            return s_matrix


    def json_set(self, __in_dict_data):
        """json_set function initiates a POST request to the targeted controller.
        It is directed at the /wm/staticentrypusher/json REST API location
        on the targeted controller.
        It also has a dictionary structure of a flow designed by the user.
        :param __in_dict_data: Input structure of a dictionary containing
        flow scheme for targeted controller.
        :return: Status of REST API call either True or False for success or
        failure, respectively.
        """
        ret = self.json_rest_call(__in_dict_data,
                                  'POST',
                                  "/wm/staticentrypusher/json")
        return ret[0] == 200

    def json_rest_call(self, __in_json_data, __in_s_action, __in_s_path):
        """json rest call function."""
        s_headers = {
            'Content-type': 'application/json',
            'Accept': 'application/json',
        }
        json_msg_body = json.dumps(__in_json_data)
        http_connect = HTTPConnection(self.s_controller_ip,
                                      self.i_controller_port)
        try:
            http_connect.request(__in_s_action,
                                 __in_s_path,
                                 json_msg_body,
                                 s_headers)
            http_response = http_connect.getresponse()
            rest_response = (http_response.status,
                             http_response.reason,
                             http_response.read())
            http_connect.close()
        except httplib.HTTPException as error:
            print(str(error))
            return None
        except httplib.NotConnected as error:
            print(str(error))
            return None
        except httplib.InvalidURL as error:
            print(str(error))
            return None
        except httplib.UnknownProtocol as error:
            print(str(error))
            return None
        except httplib.UnknownTransferEncoding as error:
            print(str(error))
            return None
        except httplib.UnimplementedFileMode as error:
            print(str(error))
            return None
        except httplib.IncompleteRead as error:
            print(str(error))
            return None
        except httplib.ImproperConnectionState as error:
            print(str(error))
            return None
        except httplib.CannotSendRequest as error:
            print(str(error))
            return None
        except httplib.CannotSendHeader as error:
            print(str(error))
            return None
        except httplib.ResponseNotReady as error:
            print(str(error))
            return None
        except httplib.BadStatusLine as error:
            print(str(error))
            return None
        return str(rest_response)
 

    def s_rand_gen_name():
        """Generate a random name.
        :parameters: Takes No parameters.
        return: random name composed of 4 sets of 4 ints sepatated by dashes.
        """
        s_rand_name = (str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       "-" + str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       "-" + str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       "-" + str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)) +
                       str(randint(0, 9)))
        return s_rand_name

    def dict_flow_builder(self,
                          __in_s_switch,
                          __in_i_port,
                          __in_i_priority=32768,
                          __in_s_name=s_rand_gen_name(),
                          __in_i_cookie=0,
                          __in_s_active="true",
                          __in_s_actions="output=flood"
                          ):
        """Create the flow dictionary."""
        dict_flow = {"switch": __in_s_switch,
                     "name": __in_s_name,
                     "cookie": __in_i_cookie,
                     "port": __in_i_port,
                     "priority": __in_i_priority,
                     "active": __in_s_active,
                     "actions": __in_s_actions
                     }
        return dict_flow


def main():
    """main function."""
    print("Starting LinkSmasher")
    start_time = time()
    LS_obj = LinkSmasher('127.0.0.1', 8080)
    s1 = '00:00:22:1e:6c:e1:c7:47' 
    s2 = '00:00:72:03:38:83:6a:43'
    s3 = '00:00:e2:0a:1f:15:a3:4a'
    matrix = LS_obj.json_get()
    print(matrix)
    g = {}
    astar_obj = astar(g)
    best_path = astar_obj.a_star(matrix, s2, s3)
    print("The best path found between s2 and s3 is .............")
    print(best_path)
    end_time = (time())-start_time
    print("The running time is:")
    print(end_time)
if __name__ == '__main__':
    try:
        main()
        exit(0)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Program Terminated by User.")
    except SystemExit:
        print("SystemExit: Program Successfully Terminated.")
    except Exception:
        print("ExplicitException: Something went really wrong.")
        print_exc()
_exit(1)
