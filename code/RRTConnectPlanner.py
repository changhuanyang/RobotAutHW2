import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        #plan.append(start_config)
        #plan.append(goal_config)
        
        #Peter's implement here, basing on Kai's implement

        #rrently the waypoints is unsorted
        #establish the goal tree
        while(1):           
            #import IPython
            #IPython.embed()
            #drop one vertex while
            v_drop = self.planning_env.GenerateRandomConfiguration()
            # compare the distance from ftree and the rtree
            i_nearst_f,v_nearst_f = ftree.GetNearestVertex(v_drop)
            i_nearst_r,v_nearst_r = rtree.GetNearestVertex(v_drop)
            ext_v_drop_to_f = self.planning_env.Extend(v_nearst_f,v_drop)
            ext_v_drop_to_r = self.planning_env.Extend(v_nearst_r,v_drop)

            if(ext_v_drop_to_f == None or ext_v_drop_to_r == None): continue
            
            dist_f = self.planning_env.ComputeDistance(ext_v_drop_to_f, v_drop)
            dist_r = self.planning_env.ComputeDistance(ext_v_drop_to_r, v_drop)

            # if the v_drop is close enough to both two tree,
            # add this vertice into both trees and then terminate.
            if(dist_f <= epsilon and dist_r <= epsilon): 
                final_vid_v_drop_r = rtree.AddVertex(v_drop)
                rtree.AddEdge(i_nearst_r,final_vid_v_drop_r)
                self.planning_env.PlotEdge(v_nearst_r,v_drop)

                final_vid_v_drop_f = ftree.AddVertex(v_drop)
                ftree.AddEdge(i_nearst_f,final_vid_v_drop_f)
                self.planning_env.PlotEdge(v_nearst_f,v_drop)
                
                break
            elif(dist_f <= dist_r): # v_drop more close to the ftree
                vid_v_drop_f = ftree.AddVertex(ext_v_drop_to_f)
                ftree.AddEdge(i_nearst_f,vid_v_drop_f)
                self.planning_env.PlotEdge(v_nearst_f,ext_v_drop_to_f)
            
            else: # v_drop more close to the rtree
                vid_v_drop_r = rtree.AddVertex(ext_v_drop_to_r)
                rtree.AddEdge(i_nearst_r,vid_v_drop_r)
                self.planning_env.PlotEdge(v_nearst_r,ext_v_drop_to_r)
        #assert(v_drop != None)
        # Terminate condition : the distance from v_drop to both ftree and rtree is less than epsilon 

        # use tree.edge one both tree to find the valid path
            
        # for ftree's path : [start_config -> v_drop) 
        plan_ftree = self.find_path(ftree, 0, final_vid_v_drop_f)
        plan_ftree.reverse()
        print "plan ftree"
        print plan_ftree            
        #assert ( len(plan_ftree) != 0 )

        # for rtree's path: [Goal_config -> v_drop) then reverse
        plan_rtree = self.find_path(rtree, 0, final_vid_v_drop_r)
        print "plan rtree"
        print plan_rtree
        #assert ( len(plan_rtree) != 0 )

        # combine ftree's path + v_drop + rtree's path
        for i in plan_ftree:
            plan.append(i)

        plan.append(v_drop)
        print "final v_drop"
        print v_drop

        for i in plan_rtree:
            plan.append(i)
        print "total plan"
        print plan
        # end of implement
        return plan
    # help function to find the route
    def find_path(self, tree, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path = []
        while(id_next_v != start_id):
            id_next_v = tree.edges[id_next_v]
            path.append(tree.vertices[id_next_v])
        return path