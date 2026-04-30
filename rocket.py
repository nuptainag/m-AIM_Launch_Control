# Trajectory calculation and intercept simulation
# *All units are metric

import math


########## Distance Simulation Functions ##########

# Bug 1 fixed: signature now accepts launchCoords dict (matching the call site in launchSystem.py)
# instead of two separate lat/lon floats.
# Bug 3 fixed: targetCoords is a GeoJSON list [lon, lat, alt], so index 0 = lon, 1 = lat.
def calculateLaunchDirection(launchCoords, targetCoords):
    
    launchStationLat_rad  = math.radians(launchCoords["lat"])
    launchStationLong_rad = math.radians(launchCoords["lon"])
    targetLat_rad  = math.radians(targetCoords[1])   # GeoJSON: index 1 = latitude
    targetLong_rad = math.radians(targetCoords[0])   # GeoJSON: index 0 = longitude
    
    deltaLong = targetLong_rad - launchStationLong_rad
    
    # Calculate vector components
    x = math.sin(deltaLong) * math.cos(targetLat_rad)
    y = math.cos(launchStationLat_rad) * math.sin(targetLat_rad) - (math.sin(launchStationLat_rad) * math.cos(targetLat_rad) * math.cos(deltaLong))
    
    # Calculate initial bearing
    initial_bearing = math.atan2(x, y)
    
    # Convert to degrees and normalize to 0-360
    initial_bearing_deg = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing_deg + 360) % 360
    
    return compass_bearing

def equirectangularDistance(launchStationLat_deg, launchStationLong_deg, targetLat_deg, targetLong_deg): # shortest distance between two points on the earth
    
    R = 6371000                                                                                 # radius of the earth in meters
    
    launchStationLat_rad, launchStationLong_rad = math.radians(launchStationLat_deg), math.radians(launchStationLong_deg)
    targetLat_rad, targetLong_rad = math.radians(targetLat_deg), math.radians(targetLong_deg)
    
    x = (targetLong_rad - launchStationLong_rad) * math.cos((launchStationLat_rad + targetLat_rad) / 2)
    y = targetLat_rad - launchStationLat_rad
    
    distance = math.sqrt(x*x + y*y) * R
    
    return distance
    
    
def runSimulation(fixedPitch_deg, launchDirection_deg, launchCoords, rocketParameters):
    
    # 1. SETUP VARIABLES
    initialMass_kg = rocketParameters["initialMass"]
    propellantMass_kg = rocketParameters["propellantMass"]
    burnTime_s = rocketParameters["burnTime"]
    avgThrust_N = rocketParameters["avgThrust"]
    Cdo = rocketParameters["Cdo"]
    area_m2 = rocketParameters["area_m2"]
    
    # Initial State
    # Bug 2 fixed: launchCoords is a dict {"lat", "lon", "alt"} — use key access throughout,
    # not integer indices. The final_range call at the bottom already used keys correctly;
    # now the top of the function and the while-loop guard match.
    currentLat    = launchCoords["lat"]
    currentLong   = launchCoords["lon"]
    currentAlt_m  = launchCoords["alt"]  # Use launch elevation, not 0
    maxAlt_m = currentAlt_m
    
    velocity_x, velocity_y = 0, 0
    dt = 0.1 
    t = 0
    
    # History Lists (For Plotting)
    rocketLats = [currentLat]
    rocketLongs = [currentLong]
    rocketAlts = [currentAlt_m]
    
    # Earth Parameters for calc
    R_earth = 6371000 
    
    # Convert Angles to Radians
    launchAngle_rad = math.radians(fixedPitch_deg)
    azimuth_rad = math.radians(launchDirection_deg) # New: Heading
    
    launchAlt_m = launchCoords["alt"]  # Bug 2 fixed: was launchCoords[2]

    # 2. FLIGHT LOOP
    # Stop if it hits the ground (below launch altitude) or max time
    while currentAlt_m >= launchAlt_m:
        
        # Air density (approximate at current altitude)
        rho = 1.225 * math.exp(-(currentAlt_m) / 8500) 
        
        # --- B. VELOCITY ---
        v_sq = velocity_x**2 + velocity_y**2
        v = math.sqrt(v_sq)
        
        if v > 0:
            theta = math.atan2(velocity_y, velocity_x)
        else:
            theta = launchAngle_rad

        # --- C. MASS & THRUST ---
        if t < burnTime_s:
            currentMass_kg = initialMass_kg - (propellantMass_kg * (t / burnTime_s))
            Thrust_Current = avgThrust_N
        else:
            currentMass_kg = initialMass_kg - propellantMass_kg
            Thrust_Current = 0
            
        Weight = 9.81 * currentMass_kg
        Drag_Current = 0.5 * rho * v_sq * Cdo * area_m2
        
        # --- D. FORCES & ACCELERATION ---
        F_axial = Thrust_Current - Drag_Current
        Fx = F_axial * math.cos(theta)
        Fy = F_axial * math.sin(theta) - Weight
        
        accel_x = Fx / currentMass_kg
        accel_y = Fy / currentMass_kg
        
        # --- E. INTEGRATION ---
        velocity_x += accel_x * dt
        velocity_y += accel_y * dt
        
        # Vertical Update
        d_alt = velocity_y * dt
        currentAlt_m += d_alt
        
        # Horizontal Update (The 3D part)
        d_horizontal = velocity_x * dt
        
        # Break horizontal distance into North and East components
        d_north = d_horizontal * math.cos(azimuth_rad)
        d_east = d_horizontal * math.sin(azimuth_rad)
        
        # Convert meters to degrees
        d_lat = (d_north / R_earth) * (180 / math.pi)
        d_long = (d_east / (R_earth * math.cos(math.radians(currentLat)))) * (180 / math.pi)
        
        currentLat += d_lat
        currentLong += d_long
        
        # Store History
        rocketLats.append(currentLat)
        rocketLongs.append(currentLong)
        rocketAlts.append(currentAlt_m)
        
        if currentAlt_m > maxAlt_m:
            maxAlt_m = currentAlt_m

        t += dt
        if t > 300: break 
    
    # Calculate final ground range
    final_range = equirectangularDistance(launchCoords["lat"], launchCoords["lon"], currentLat, currentLong)
            
    return final_range, maxAlt_m, rocketLats, rocketLongs, rocketAlts
    

def checkRange(rocketParameters, launchCoords, targetCoords, launchAngle_deg, launchDirection_deg):

    # Bug 3 fixed: targetCoords is a GeoJSON list [lon, lat, alt] — use integer indices,
    # not dict keys. index 0 = longitude, 1 = latitude, 2 = altitude.
    targetLat_deg = targetCoords[1]   # GeoJSON: index 1 = latitude
    targetLong_deg = targetCoords[0]  # GeoJSON: index 0 = longitude
    targetAlt_m = targetCoords[2]

    targetRange_m = equirectangularDistance(launchCoords["lat"], launchCoords["lon"], targetLat_deg, targetLong_deg)
    maxRange_m, maxAlt_m, rLats, rLongs, rAlts = runSimulation(launchAngle_deg, launchDirection_deg, launchCoords, rocketParameters)
    
    inRange = True if maxRange_m >= targetRange_m and maxAlt_m >= targetAlt_m else False
    
    return inRange