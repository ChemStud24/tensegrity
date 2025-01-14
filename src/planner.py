import numpy as np
import pickle
import rospy
import rosnode
from tensegrity.msg import State, Action
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse, GetBarHeight

class MotionPlanner:

	def __init__(self):
		sub_topic = '/state_msg'
		pub_topic = '/action_msg'
		self.sub = rospy.Subscriber(sub_topic,State,self.callback)
		self.pub = rospy.Publisher(pub_topic,Action,queue_size=10)

		# load state-action transition dictionary
		with open('../calibration/motion_primitives.pkl','rb') as f:
			self.action_dict = pickle.load(f)

		# ordered list of motion primitives
		self.primitives = ['100_100','120_120','140_140','100_120','120_100','100_140','140_100','120_140','140_120','ccw','cw']

		# define start, goal, and obstacles

		# run A star planner for the known start, goal, and obstacles
		self.Astar()

		self.count = 0

		# determine if the tracking service is available for feedback
		if '/tracking_service' in rosnode.get_node_names():
			self.closed_loop = True
		else:
			self.closed_loop = False

	def callback(self,msg):
		# receiving a state message means the robot is ready for the next action
		print('Action ' + str(self.count))

		# if tracking is available
		if self.closed_loop:
			# get pose to check if we deviated from the plan
			COM,principal_axis,_ = self.get_pose()
		
			# if we did, re-run the planner
			if False: # replace this with a reasonable condition
				self.Astar()

		# publish results
		action_msg = Action()
		for act in self.action_sequence:
			action_msg.actions.append(act)
		self.pub.publish(action_msg)
		self.action_sequence.pop(0)
		self.count += 1

	def Astar(self):
		# put A star planning code here
		# as a stand-in, we just do ccw 15 times
		plan = [9 for x in range(15)]
		self.action_sequence = [self.primitives[p] for p in plan]

	def run(self, rate):
		while not rospy.is_shutdown():
			rate.sleep()

	def get_pose(self):
		service_name = "get_pose"
		
		vectors = np.array([[0.0,0.0,0.0]])
		centers = []
		endcaps = []
		try:
			request = GetPoseRequest()
			get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
			# rospy.loginfo("Request sent. Waiting for response...")
			response: GetPoseResponse = get_pose_srv(request)
			# rospy.loginfo(f"Got response. Request success: {response.success}")
			if response.success:
				for pose in response.poses:
					
					rotation_matrix = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w]).as_matrix()
					
					unit_vector = rotation_matrix[:,2]
					center = [pose.position.x,pose.position.y,pose.position.z]
					endcaps.append(np.array(center) + L/2*unit_vector)
					endcaps.append(np.array(center) - L/2*unit_vector)
					
					centers.append(center)
					vectors += unit_vector
			COM = np.mean(np.array(centers),axis=0)
			principal_axis = vectors/np.linalg.norm(vectors)
			endcaps = np.array(endcaps)/100 # convert to meters
			# reformat COM and PA
			COM = np.reshape(COM[0:2],(2,1))
			principal_axis = principal_axis[:,0:2]
			principal_axis = np.reshape(principal_axis,(2,1))
		except rospy.ServiceException as e:
			rospy.loginfo(f"Service call failed: {e}")
		return COM, principal_axis, endcaps


if __name__ == '__main__':
	rospy.init_node('motion_planner')
	planner = MotionPlanner()
	rate = rospy.Rate(30)
	planner.run(rate)