import numpy as np

dt = 0.1

def cubic_spiral(theta_i, theta_f,n=10):
    '''
    Returns thetas corresponding to a cubic turn given initial and final theta. 
    'n' is the measure of smoothness.
    '''
    x = np.linspace(0, 1, num=n)
    return (theta_f-theta_i)*(-2*x**3 + 3*x**2) + theta_i
    
def straight(dist, curr_pose, radius, num_st_pts = 10):
    '''
    Returns poses for a straight path given current pose and distance. 
    radius is a dummy parameter. 'num_st_pts' is the number of sampling points.
    '''
    x0, y0, t0 = curr_pose
    xf, yf = x0 + dist*np.cos(t0), y0 + dist*np.sin(t0)
    x = (xf - x0) * np.linspace(0, 1, num_st_pts) + x0
    y = (yf - y0) * np.linspace(0, 1, num_st_pts) + y0
    return x, y, t0*np.ones_like(x)

def turn(change, curr_pose, radius = 0.5, num_turn_pts = 50):
    '''
    Returns poses for a turn given current pose and angle. 
    radius is the radius of the turn: (radius<0.5) set to maximum 0.5 by default. 
    'num_turn_pts' is the number of sampling points.
    '''
    x0, y0, t0 = curr_pose
    theta = cubic_spiral(t0, t0 + np.deg2rad(change), num_turn_pts)
    x= x0 + np.cumsum(np.cos(theta)*dt*radius/3)
    y= y0 + np.cumsum(np.sin(theta)*dt*radius/3)
    return x, y, theta

def generate_trajectory(route, init_pose = (0, 0,np.pi/2),radius = 0.5):
    '''
    Returns a 3xn ndarray of poses as columns for a given set of commands as [manouevre,value] and the initial pose. 
    radius is the radius of the turn: (radius<0.5) set to maximum 0.5 by default. 
    '''
    curr_pose = init_pose
    func = {'straight': straight, 'turn': turn}
    x, y, t = np.array([]), np.array([]),np.array([])
    for manoeuvre, command in route:
        px, py, pt = func[manoeuvre](command, curr_pose, radius)
        curr_pose = px[-1],py[-1],pt[-1]
        x = np.concatenate([x, px])
        y = np.concatenate([y, py])
        t = np.concatenate([t, pt])
        
    return (np.vstack([x, y, t])).T

def get_route(path,radius = 0.5):
    '''
    Returns an array of commands as [manouevre,value] for a given set of path coordinates.
    radius is the radius of the turn: (radius<0.5) set to maximum 0.5 by default. 
    '''
    path_list = np.array(path) 
    route = []
    segment = ['straight',1-radius]
  
    for i in range(len(path_list)-2):
        x1,y1 = path_list[i][0],path_list[i][1]
        x2,y2 = path_list[i+1][0],path_list[i+1][1]
        x3,y3 = path_list[i+2][0],path_list[i+2][1]
        check  = ((x1-x2)*(y3-y2) - (x3-x2)*(y2-y1))
        if check == 0:
            segment[1] += 1

        else:
            route.append(tuple(segment))
            segment[0] = 'turn'
            
            if check>0:
                if x1 == x2:
                    segment[1] = 90
                else:
                    segment[1] = -90   
            else:
                if x1 == x2:
                    segment[1] = -90 
                else:
                    segment[1] = 90
          
        route.append(tuple(segment))
        segment = ['straight',1-2*radius]  
    
    segment[1] += radius
    route.append(tuple(segment))
    return route      
