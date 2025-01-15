"""
implementation of the Vector Field Histogram + algorithm
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import random

DEFAULT_THRESH = 1 # distance threshold used to convert into a binary histogram
MAX_ANGLE = 180 # total range of the scan (degrees)
MIN_ANGLE = 6 # angle difference between laser beams (degrees)
TOTAL_RAYS = 31 # total of laser beams
ROBOT_RADIUS = 0.45 # radius of the robot in meters
MIN_VALLEY_ANGLE = math.pi/15 # aperture necesary to consider a valley due to the size of the robot
SAFETY_DIST = 0.5 # safety distance from obstacles

class VFHPlus:
    def __init__(self, laser_readings, min_angle = MIN_ANGLE, max_angle = MAX_ANGLE, last_dir=None, safety_distance=SAFETY_DIST):
        """
        Constructor for the VFHPlus class.

        :param laser_readings: Array with laser readings (distances to obstacles).
        :param safety_distance: Minimum distance to consider a cell as an obstacle (in meters).
        :param threshold: Density threshold to define blocked regions.
        """
        self.laser_readings = np.array(laser_readings)
        self.safety_distance = safety_distance
        self.threshold = self.safety_distance + 0.25
        self.max_angle = max_angle  # Default maximum angle for laser readings [degrees]
        self.min_angle = min_angle  # Angle gap between beams [degrees]
        self.min_valley_length = int(MIN_VALLEY_ANGLE // np.radians(self.min_angle)) # scalar int
        self.last_dir = last_dir # last steering direction chosen during VFH+ run
        self.direction = 0 # best steering direction in radians
        self.hist = None # polar histogram
        self.masked_hist = [] # masked polar histogram
        self.radio_enlargement = ROBOT_RADIUS # radio enlargement of the obstacles
        self.kdir = 0.75 # constant for cost function to ponder direction diff
        self.ktarget = 0.5 # cosntant for cost function to ponder target 
        

    def getDirection(self): # returns in radians the steering direction selected
        if self.direction is not None:		
            return math.radians(self.direction)
        return None
        
    def getMaskedHist(self): # returns the masked polar histogram
        return self.masked_hist  
          		
    def setThreshold(self, value=None): # sets the threshold for the masked polar histogram
        if value is None:
            self.threshold = np.mean(self.laser_readings) # mean value of the distances read by default
        self.threshold = value # if given a value set it
    
    def filter_data(self):
        """
        Filters the distances stored in self.laser_reading to reduce noise and replace incorrect readings with Z_MAX.
        Uses the mean of a 5-reading neighborhood as the expected value for each reading.

        :return: A filtered list of distances where invalid readings are replaced by a neighbourhood value.
        """
        # Parameters for the sensor model
        LAMBDA = 0.5  # Parameter for obstacle model
        Z_MAX = float('inf')  # Maximum sensor range
        B = 0.5       # Standard deviation for Gaussian

        def noisy(z, z_exp):
            """Gaussian noise model."""
            p1 = 1 / (math.sqrt(2 * math.pi * B))
            elevate = (-0.5 * math.pow(z - z_exp, 2)) / B
            p2 = math.exp(elevate)
            return p1 * p2

        def obstacle(z):
            """Close or unexpected obstacle model."""
            return LAMBDA * math.exp(-LAMBDA * z)

        def sensor_model(z, z_exp):
            """Combines Gaussian noise, obstacle, and uniform components into a sensor model."""
            prob = 0.0
            
            # Obstacles
            if z < z_exp:
                prob += obstacle(z)
            
            # Gaussian noise
            prob += noisy(z, z_exp)

            # Uniform component
            prob += 1 / Z_MAX

            # Maximum reading
            if z == Z_MAX:
                prob = 1

            return prob

        # Threshold for filtering (keep readings with probabilities >= threshold)
        PROB_THRESHOLD = 0.5

        # Neighborhood size for the moving average
        NEIGHBORHOOD = 10
        half_window = NEIGHBORHOOD // 2

        filtered_readings = []
        last_z = Z_MAX
        for i in range(len(self.laser_readings)):
            # Compute z_expected as the mean of a 5-reading neighborhood
            start_idx = max(0, i - half_window)
            end_idx = min(len(self.laser_readings), i + half_window + 1)
            z_expected = np.mean(self.laser_readings[start_idx:end_idx])

            # Evaluate the probability of the current reading
            z = self.laser_readings[i]
            prob = sensor_model(z, z_expected)

            if prob >= PROB_THRESHOLD:
                filtered_readings.append(z)  # Keep valid reading
            else:
                filtered_readings.append(last_z)  # Replace invalid reading with the expected neighbour
            # store the last neighbour
            last_z = z_expected
        return np.array(filtered_readings)

        
    def compute_histogram(self):
        """
        Calculates a polar histogram based on laser readings.
        """
        histogram = []
        numRays = len(self.laser_readings)
        self.laser_readings = self.filter_data()
        for i in range(numRays): 
            magnitude = self.laser_readings[i] # save the distance to the object
            # theta is the angle of the laser beam respect to the forward direction of movement
            if i < numRays/2:
                theta = (self.max_angle/2) - i*self.min_angle  
            else:
                theta = ((numRays//2)-i) * self.min_angle
            # add to the histogram
            histogram.append((magnitude, theta))
        # convert to numpy array of tuples (distance, theta)
        self.hist = np.array(histogram)

    def compute_masked_hist(self):
        """
        Computes the masked polar histogram (binary values) using a threshold
        -> 1 if there is obstacle
        -> 0 if it is clear
        Enlarges the obstacles in function of their closeness to the robot 
        """
        masked_hist = []
        #self.setThreshold(1) # set the threshold to transform into a binary histogram
        for e in self.hist:
            d, theta = e
            v = 1 if d <= self.threshold else 0
            masked_hist.append((v,theta))
        # convert to numpy array
        masked_hist = np.array(masked_hist, dtype=object)
        
        self.masked_hist = masked_hist.copy()
        # enlarge the obstacles according to their distance to the robot
        for i,e in enumerate(self.hist):
            # i is the index of the element e which is a tuple (magnitude, theta)
            d, theta = e
            # apply enlargement of obstacles if they are close enough (at a dist between [radio_enlargement, threshold])
            alpha = 0
            if d >= self.radio_enlargement and d < self.threshold:
                alpha = math.asin(self.radio_enlargement / d)
            # get the binary value for the masked histogram
            v = 1 if d <= self.threshold else 0
            # min angle between beams in radians
            min_angle_rads = math.radians(self.min_angle)
            # determine the neighbourhood covered by alpha
            if abs(alpha) > min_angle_rads:
                j = int(abs(alpha) // min_angle_rads)
                # forward enlargement
                for u in range(1, j + 1):
                    forward_idx = (i + u) % len(self.masked_hist)  # Wrap around if necessary
                    if self.masked_hist[forward_idx][0] == 0:  # Only enlarge if it's clear
                        self.masked_hist[forward_idx] = (1, self.masked_hist[forward_idx][1])
                
                # Backward enlargement
                for u in range(1, j + 1):
                    backward_idx = (i - u) % len(self.masked_hist)  # Wrap around if necessary
                    if self.masked_hist[backward_idx][0] == 0:  # Only enlarge if it's clear
                        self.masked_hist[backward_idx] = (1, self.masked_hist[backward_idx][1])
                
            # update the current index
            self.masked_hist[i] = (v,theta)    
        # overwrite the enlarged binary polar histogram
        #self.masked_hist = hist.copy()

    def find_valleys(self):
        """
        Finds the "valleys" in the masked polar histogram, representing safe directions as they are free of obstacles
        """
        valley_list = [] # list of valleys 
        valley = [] # list of indexes which involve a valley
        for i, element in enumerate(self.masked_hist):
            # consider the different valleys and safe them using a tuple of index (begin, end)
            v, _ = element
            if v < 1: # empty space
                valley.append(i)
            else:
                if len(valley) > 0:
                    valley_list.append(valley)
                    valley = []
        # append the last valley 
        if len(valley) > 0:
            valley_list.append(valley) 
        # return None if list is empty
        return valley_list if len(valley_list) > 0 else None

    def select_direction(self, valleys, target):
        """
        Selects the best direction based on available valleys and the goal direction.

        :param valleys: List of angular indices of safe valleys.
        :param goal_angle: Desired goal angle (in degrees).
        :return: Best direction in radians, constrained to [-MAX_ANG/2, +MAX_ANG/2] degrees.
        """
        valleys = self.find_valleys()
        if valleys is None:
            return None  # No safe directions available
        # compute the best direction of a valley using cost function
        if self.last_dir is None:
            self.last_dir = 0 # suppose the last dir was going forward with no steering angle
        g_star = float('inf') # initially infinite
        for v in valleys: # consider the valleys with an angle minium of 
            n = len(v)
            if n > self.min_valley_length: # if there is a valley of at least 30 degrees
                for index in v:
                    direction = self.masked_hist[index][1]
                    g = self.ktarget * target + self.kdir * abs(direction-self.last_dir)
                    if g < g_star:
                        g_star = g
                        self.direction = direction

    def compute_direction(self, target=0.0):
        """
        Calculates the movement direction for the robot using the VFH+ algorithm.

        :param goal_angle: Desired goal angle (in degrees).
        :return: Selected angular direction (in degrees) or None if movement is not possible.
        """
        self.compute_histogram()
        self.compute_masked_hist()
        valleys = self.find_valleys()
        self.select_direction(valleys, target)
        

    def plot_histogram(self, hist, str="polar_histogram"):
        # Extract magnitude and theta values
        magnitudes = hist[:, 0]
        angles = hist[:, 1]

        # Plot the histogram using hist functions of pyplot
        plt.ion()
        plt.clf()
        plt.hist(angles, bins=len(self.laser_readings), weights=magnitudes, edgecolor='black', alpha=0.7, label="Laser Magnitudes")
        plt.axhline(y=self.threshold, color='red', linestyle='--', linewidth=1.0, label="Threshold of direction blocking")
        plt.xlabel("Theta (degrees)")  # Label for x-axis
        plt.ylabel("Magnitude")  # Label for y-axis
        plt.title("Polar Histogram of Laser Readings")
        plt.xlim(-self.max_angle/2,+self.max_angle/2)
        plt.grid(True)  # Show a grid for better visualization
        plt.legend()  # Add a legend
        
        #plt.show()
        plt.draw()
        plt.pause(0.1)
        
        str += ".png"
        #plt.savefig(str)

"""
# Example usage
# Initialize laser readings (in meters)
laser_readings = np.full(TOTAL_RAYS, 0.0)
for i in range(len(laser_readings)):
    if i < len(laser_readings) // 2:
        laser_readings[i] = 0.5
    else:
        rand = random.randint(5,10)*random.random()
        v = rand if rand > 0 else 0.1
        laser_readings[i] = v
# Create an instance of VFHPlus
vfh = VFHPlus(laser_readings)
# start the algorithm
vfh.compute_direction()
"""

