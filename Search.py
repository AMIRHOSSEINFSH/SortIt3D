import heapq

import State
from Solution import Solution
from Problem import Problem
from datetime import datetime
from collections import deque
from typing import Set
from heapq import heappush, heappop


class Search:

    @staticmethod
    def bfs(prb: Problem) -> Solution:  # this method get a first state of Problem and do bfs for find solution if no
        # solution is find return None else return the solution
        start_time = datetime.now()
        queue = []
        state = prb.initState

        queue.append(state)
        while len(queue) > 0:
            state = queue.pop(0)
            neighbors = prb.successor(state)
            for c in neighbors:
                if prb.is_goal(c):
                    print(c.prev_action)
                    # print(prb.get_cost_from_change(c, c.prev_action[0]))
                    return Solution(c, prb, start_time)
                queue.append(c)
        return None

    @staticmethod
    def dfs(prb: Problem, max_depth: int) -> Solution:
        start_time = datetime.now()
        stack = deque()
        visited: Set[int] = set()

        state = prb.initState
        stack.append((state, 0))

        while stack:
            state, depth = stack.pop()

            if state.__hash__() in visited:
                continue

            visited.add(state.__hash__())

            if prb.is_goal(state):
                return Solution(state, prb, start_time)

            if depth < max_depth:
                for neighbor in prb.successor(state):
                    if neighbor.__hash__() not in visited:
                        stack.append((neighbor, depth + 1))

        return None

    @staticmethod
    def iterative_deepening(prb: Problem, max_depth: int) -> Solution:
        for depth_limit in range(1, max_depth + 1):
            solution = Search.dfs(prb, depth_limit)
            if solution:
                print("find in level of: " + str(depth_limit))
                return solution

        return None

    @staticmethod
    def ids(prb: Problem) -> Solution:
        start_time = datetime.now()
        max_depth = 0

        while True:
            stack = [prb.initState]
            visited: Set[int] = set()

            while stack:
                state = stack.pop()
                # visited.add(state)

                if prb.is_goal(state):
                    return Solution(state, prb, start_time)

                if state.__hash__() in visited:
                    continue

                visited.add(state.__hash__())

                if max_depth == 0:
                    continue

                if state.g_n >= max_depth:
                    continue

                for neighbor in prb.successor(state):
                    if neighbor.__hash__() not in visited:
                        stack.append(neighbor)

            max_depth += 1

    @staticmethod
    def ucs(prb: Problem) -> Solution:
        start_time = datetime.now()
        queue = []
        state = prb.initState
        visited: Set[int] = set()

        heapq.heappush(queue, (0, state))

        while queue:
            _, state = heapq.heappop(queue)

            visited.add(state.__hash__())

            if prb.is_goal(state):
                return Solution(state, prb, start_time)

            for neighbor in prb.successor(state):

                if prb.is_goal(neighbor):
                    return Solution(neighbor, prb, start_time)

                newCost = neighbor.cost
                try:
                    newCost += prb.get_cost_from_change(neighbor, neighbor.prev_action[0])
                except:
                    pass

                if neighbor.__hash__() not in visited:
                    neighbor.cost = newCost
                    # cost = (neighbor.g_n * (neighbor.g_n + 1)) / 2
                    heapq.heappush(queue, (newCost, neighbor))
                elif newCost < neighbor.cost:
                    neighbor.cost = newCost
                    heapq.heappush(queue, (newCost, neighbor))

        return None
