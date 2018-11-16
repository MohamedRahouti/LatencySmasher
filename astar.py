"""generic A-Star path searching algorithm."""
from heapq import heappush, heappop
from os import _exit
from sys import exit
from traceback import print_exc
from graph import Graph


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
        #print("All paths between "+start+" and "+end+" are:")
        #print(paths)
        return paths

    def fit_matrix(self, matrix, start, end, path=[]):
        path = path + [start]
        #if start == end:
        #    print("Source and destination are the same")
        #    return [path]
        #paths = []
        """Construct a graph (dictionary)first."""
        graph = {}
        g_matrix = matrix[0][2]
        for i in range(len(g_matrix)):
            #print("The current dpid is: %s" %g_matrix[i][0][0])
            cur_dpid = g_matrix[i][0][0]
            cur_path = []
            for j in range(len(g_matrix[i])):
                if (g_matrix[i][j][2] != 0 and g_matrix[i][j][3] != 0):
                    cur_path.append(g_matrix[i][j][1])
                    cur_path.append(g_matrix[i][j][3])
            if len(cur_path) != 0:
                #print("The current path is:")
                #print(cur_path)
                graph[cur_dpid] = cur_path
        print("The resulting graph after transformation is...............")
        print(graph)
        #list_paths = []
        #print("The list of all paths between "+start+ " and "+end)
        #paths = self.find_all_paths(graph, start, end)
        #print(paths)
        return graph

    def find_hs(self, gr, paths, src, dst):
        graph_h = []
        #graph = Graph(gr)
        #paths = graph.find_all_paths2(src, dst)
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
                    h += self.find_g(gr, paths[i][j], paths[i][j+1]) + \
                                     graph_h[i][prev_h_indice]
                    graph_h[i].append(h)
                    prev_h_indice += 1
                    j -= 1
        return graph_h

    def find_h(self, gr, src, dst):
        graph_h = {}
        graph = Graph(gr)
        paths = graph.find_all_paths(src, dst)
        for i in range(len(paths)):
            h = 0
            if len(paths[i]) == 2:
                graph_h[paths[i][1]] = h
            else:
                j = len(paths[i])-2
                while j > 0:
                    expanded_node = paths[i][j]
                    list_by_key = gr[paths[i][j]]
                    k = 0
                    while k < len(list_by_key):
                        if list_by_key[k] == paths[i][j+1]:
                            h += list_by_key[k+1]
                            graph_h[expanded_node] = h
                            break
                        k += 2
                    j -= 1
        return graph_h

    def find_g(self, g, node1, node2):
        list_by_key = g[node1]
        gn = 0
        i = 0
        while i < (len(list_by_key)):
            if list_by_key[i] == node2:
                gn = list_by_key[i+1]
                break
            i += 2
        return gn

    def find_neighbors(self, current_dpid):
        print("Searching for neighbors.................")
        neighbors = []
        k = 0
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix)):
                if self.matrix[i][j][0] == current_dpid and self.matrix[i]\
                   [j][0] != self.matrix[i][j][1]:
                    neighbors.append([])
                    neighbors[k].append(self.matrix[i][j][0])
                    neighbors[k].append(self.matrix[i][j][1])
                    neighbors[k].append(self.matrix[i][j][2])
                    neighbors[k].append(self.matrix[i][i][3])
                    k += 1
        print("Here are the neighbors of %s :" % current_dpid)
        print(neighbors)
        return neighbors

    def remove_nodes(self, neighbors, void_nodes):
        for node in range(len(void_nodes)):
            for i in range(len(neighbors)):
                if neighbors[i][1] == void_nodes[node]:
                    neighbors.pop(i)
                    break
        return neighbors

    def a_star(self, matrix, start_dpid, goal_dpid):
        if self.is_goal_reached(start_dpid, goal_dpid):
            print("Source and Destination are the same")
            return None
        #graph = Graph(g)
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

        """
        edge_dpid = 'none'
        void_nodes = []
        current_dpid = start_dpid
        while count < len(openSet):
            print("The length of openSet is: %s" %len(openSet))
            print("The current dpid is: %s" %current_dpid)
            if self.is_goal_reached(current_dpid, goal_dpid):
                print("Path successfully found! Good job!!")
                return path
            neighbors = self.find_neighbors(current_dpid)
            fscores = []
            findex = 0
            no_childs = 0
            if len(neighbors) != 0:
                neighbors = self.remove_nodes(neighbors, void_nodes)
            if len(neighbors) != 0:
                for index in range(len(neighbors)):
                    if (neighbors[index][1] != edge_dpid):
                        current_g = neighbors[index][2]
                        next_n = neighbors[index][1]
                        heuristic = neighbors[index][3]
                        print("The gscore is: %s" %current_g)
                        print("The heuristic cost h(n) is: %s" %heuristic)
                        fscores.append([])
                        fscores[findex].append(next_n)
                        fscores[findex].append(int(current_g) + int(heuristic))
                        fscores[findex].append(int(current_g))
                        no_childs = 1
                        findex += 1
            if no_childs == 1:
                min_score = fscores[0][1]
                current_g = fscores[0][2]
                next_move_dpid = fscores[0][0]
                for score in range(len(fscores)):
                    if min_score > fscores[score][1]:
                        min_score = fscores[score][1]
                        current_g = fscores[score][2]
                        next_move_dpid = fscores[score][0]
                path.append(next_move_dpid)
                openSet.append(next_move_dpid)
                closedSet.append([])
                closedSet[len(closedSet)-1].append(current_dpid)
                closedSet[len(closedSet)-1].append(next_move_dpid)
                closedSet[len(closedSet)-1].append(current_g)
                closedSet[len(closedSet)-1].append(min_score)
                print("The closed List is:")
                print(closedSet)
                print("The best next move will be via: %s" %next_move_dpid)
                count += 1
                current_dpid = next_move_dpid
            else:
                closedSet.pop()
                path.pop()
                edge_dpid = current_dpid
                current_dpid = closedSet[len(closedSet)-1][1]
                void_nodes.append(edge_dpid)
                print("The list of nodes must be avoided in path \
                      reconstruction:")
                print(void_nodes)
                print("The closed List is:")
                print(closedSet)
        print("The shortest path found is:")
        return None"""

    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    """main function."""

    def random_graph(self):
        graph = {}
        for i in range(1000):
            #node_id = random.randint(10, 1000)
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


def main():
    print("Starting A* Search")
    g1 = {"s": ["a", 1, "b", 3, "c", 10],
         "a": ["d", 5, "s", 1],
         "b": ["e", 4, "s", 3],
         "c": ["e", 6, "s", 10],
         "e": ["g", 7, "b", 4, "c", 6],
         "d": ["f", 2, "g", 3, "a", 5],
         "g": ["e", 7, "d", 3],
         "f": ["d", 2]}
    """
    myObject = astar(g1)
    graph = Graph(g1)
    s_src = 's'
    s_dest = 'g'
    print('All paths from vertex ' + s_src + ' to vertex ' + s_dest + ':')
    #paths = graph.find_all_paths(s_src, s_dest)
    paths = myObject.find_all_paths(g1, s_src, s_dest)
    print(paths)
    best_path = myObject.a_star(g1, s_src, s_dest)
    list_hs = myObject.find_hs(g1, s_src, s_dest)
    print("Here is the 2 D list of heuristics:")
    print(list_hs)
    print("The best path from " + s_src + " to " + s_dest + " is:")
    print(best_path)
    #print("The random Graph Generator:")
    #print(myObject.random_graph())
    #print(random.randint(0, 9))
    #print(random.choice('ABCD'))
    """
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