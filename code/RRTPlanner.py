import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        #establish the start tree
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        plan.append(goal_config)
        # Kai's implementation

        #currently the waypoints is unsorted
        #establish the goal tree
        tree2 = RRTTree(self.planning_env, goal_config)
        i=0
        while(1):           
            #import IPython
            #IPython.embed()
            #drop one valid random point that only connect with goal tree
            point_drop = self.planning_env.GenerateRandomConfiguration()
            index2, dist2 = tree2.GetNearestVertex(point_drop)
            point_chosen = self.planning_env.Extend(tree2.vertices[index2], point_drop)
            vid2 = tree2.AddVertex(point_chosen)
            tree2.AddEdge(index2, vid2)
            self.planning_env.PlotEdge(tree2.vertices[index2],point_chosen)
            plan.insert(len(plan)-1-i,point_chosen)

            #Use the point_chosen as target, try to connect start tree with the target
            index, dist = tree.GetNearestVertex(point_chosen)
            point_reach = self.planning_env.Extend(tree.vertices[index], point_chosen)
           
            vid = tree.AddVertex(point_reach)
            tree.AddEdge(index, vid)
            self.planning_env.PlotEdge(tree.vertices[index],point_reach)
            i+=1
            plan.insert(i,point_reach)
            
            if (abs(point_reach[0]-point_chosen[0])<100*epsilon and abs(point_reach[1]-point_chosen[1])<100*epsilon):
                break


        return plan
        
