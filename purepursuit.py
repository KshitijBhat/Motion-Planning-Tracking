import numpy as np
import matplotlib.pyplot as plt
import trajectory

#################################################################

vmax = 0.5
goal_threshold = 0.05
lookahead = 2.0

#################################################################


def simulate_unicycle(pose, v,w, dt=0.1):
    x, y, t = pose
    return (x + v*np.cos(t)*dt, y + v*np.sin(t)*dt, t+w*dt)

class PurePursuitTracker(object):
  
    def __init__(self, x, y, v, lookahead = 3.0):
        """
        Tracks the path defined by x, y at velocity v
        x and y must be numpy arrays
        v and lookahead are floats
        """
        self.length = len(x)
        self.ref_idx = 0 #index on the path that tracker is to track
        self.lookahead = lookahead
        self.x, self.y = x, y
        self.v, self.w = v, 0

    def update(self, xc, yc, theta):
        """
        Input: xc, yc, theta - current pose of the robot
        Update v, w based on current pose
        Returns True if trajectory is over.
        """
        #Calculate ref_x, ref_y using current ref_idx

        if self.ref_idx >= self.length:
          ref_x,ref_y = self.x[-1],self.y[-1]
        else:  
          ref_x,ref_y = self.x[self.ref_idx],self.y[self.ref_idx]
        
        
        #Check if we reached the end of path, then return TRUE
        #Two conditions must satisfy
        #1. ref_idx exceeds length of traj
        #2. ref_x, ref_y must be within goal_threshold
        # Write your code to check end condition


        if self.ref_idx > self.length and np.sqrt((ref_x-self.x[-1])**2+ (ref_y-self.y[-1])**2) < goal_threshold: 
          return True           
        
        
        #End of path has not been reached
        #update ref_idx using np.hypot([ref_x-xc, ref_y-yc]) < lookahead
        
        if np.sqrt((ref_x-xc)**2+ (ref_y-yc)**2) < lookahead:
          self.ref_idx += 1 
        
        #Find the anchor point
        # this is the line we drew between (0, 0) and (x, y)

        anchor = np.asarray([ref_x - xc, ref_y - yc])

        #This is drawn from current robot pose
        #we have to rotate the anchor to (0, 0, pi/2)
        theta = np.pi/2 - theta
        rot = np.asarray([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        anchor = np.dot(rot, anchor)
        
        L = np.sqrt(anchor[0] ** 2 + anchor[1] **2) # dist to reference path
        
        X = anchor[0] #cross-track error
       
        #formula for omega
        self.w = -2*self.v*X/L/L

        return False



def main():
  x,y,t = trajectory.generate_trajectory([("straight",5),("turn",90),("straight",5),("turn",-90),("straight",5)],radius = 2.5)
  tracker = PurePursuitTracker(x,y,vmax) 
  pose = 0, 0, np.pi/2 #arbitrary initial pose
  x0,y0,t0 = pose # record it for plotting
  traj =[]

  while True:
      print("v = ",tracker.v," w = ",tracker.w)
      pose = simulate_unicycle(pose,tracker.v,tracker.w)
      
      if tracker.update(*pose):
          print("ARRIVED!!")    
          break
      traj.append([*pose, tracker.w, tracker.ref_idx])  
  
  xs,ys,ts,ws,ids = zip(*traj)
  plt.figure()
  plt.plot(x,y,label='Reference')
  plt.quiver(x0,y0, np.cos(t0), np.sin(t0),scale=12)
  plt.plot(xs,ys,label='Tracked')
  x0,y0,t0 = pose
  plt.quiver(x0,y0, np.cos(t0), np.sin(t0),scale=12)
  plt.title('Pure Pursuit trajectory')
  plt.legend()
  plt.grid()     
  plt.show()
    
if __name__ == '__main__':
  main()    



    
