import time

import networkx as nx

from algorithm.generic_sr import GenericSR
from algorithm.segment_routing.sr_utility import SRUtility


class WeightedShortestPath(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict, waypoints: dict = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        self.__nodes = nodes
        self.__links = links  # list with [(i,j,c)]
        if waypoints is not None:
            segmented_demands = SRUtility.get_segmented_demands(waypoints, demands)
            self.__demands = {idx: (s, t, d) for idx, (s, t, d) in enumerate(segmented_demands)}  # dict {idx:(s,t,d)}
            self.__segments = waypoints  # dict with {idx:(p,q)}
        else:
            self.__demands = {idx: (s, t, d) for idx, (s, t, d) in enumerate(demands)}
            self.__segments = {idx: [(p, q)] for idx, (p, q, _) in enumerate(demands)}

        self.__weights = weights
        self.__all_shortest_paths_generators = dict()
        self.__all_shortest_paths = dict()
        self.__nx_graph = nx.DiGraph()
        self.__flow_sum = dict()

        self.__create_nx_graph()
        self.__init_flow_sum_map()
        return

    def __create_nx_graph(self):
        for i, j, c in self.__links:
            w = self.__weights[i, j]
            self.__nx_graph.add_edge(i, j, weight=w, capacity=c)
        return

    def __get_all_shortest_paths_generator(self):
        for s in self.__nx_graph.nodes:
            for t in self.__nx_graph.nodes:
                if s == t:
                    continue
                self.__all_shortest_paths_generators[s, t] = nx.all_shortest_paths(
                    self.__nx_graph, source=s, target=t, weight='weight')
        return

    def __init_flow_sum_map(self):
        for i, j, _ in self.__links:
            self.__flow_sum[(i, j)] = 0
        return

    def __add_demand_val_to_path(self, path: list, demand: float):
        for idx in range(len(path) - 1):
            i = path[idx]
            j = path[idx + 1]

            self.__flow_sum[(i, j)] += demand
        return

    def __add_demand_update_objective(self, src, dst, demand):
        if (src, dst) not in self.__all_shortest_paths:
            self.__all_shortest_paths[src, dst] = list(self.__all_shortest_paths_generators[src, dst])

        shortest_paths = self.__all_shortest_paths[src, dst]
        for path in shortest_paths:
            self.__add_demand_val_to_path(path, demand)
        return

    def solve(self):
        self.__get_all_shortest_paths_generator()

        for idx, (s, t, d) in self.__demands.items():
            self.__add_demand_update_objective(s, t, d)

        paths = dict()
        for i, j, _ in self.__links:
            paths[(i, j)] = []

        for idx, (s, t, d) in self.__demands.items():
            shortest_paths = list(self.__all_shortest_paths_generators[s, t])
            path_costs = []
            for path in shortest_paths:
                path_cost = 0
                for i in range(len(path) - 1):
                    path_cost += self.__weights[path[i], path[i+1]]
                path_costs.append((path_cost, path))

            path = sorted(path_costs, key=lambda x: x[0])[0][1]
            for i in range(len(path) - 1):
                link = (path[i], path[i+1])
                paths[link].append(idx)

        return paths
