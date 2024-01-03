
//Waypoints
// ##############################################################################################################################################
#define WAYPOINT_DIST_TOLERANE  10   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
//#define NUMBER_WAYPOINTS 4          // enter the numebr of way points here (will run from 0 to (n-1))
//#define NUMBER_WAYPOINTS 2 
int waypointNumber = 0;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
//CBC
//waypointClass waypointList[NUMBER_WAYPOINTS] = { waypointClass(41.60820, -70.93200), waypointClass(41.60780, -70.93192), waypointClass(41.60774, -70.93237), waypointClass(41.60812, -70.93250)};

//Ashley School
//waypointClass waypointList[NUMBER_WAYPOINTS] = { waypointClass(41.67538, -70.93361), waypointClass(41.67565, -70.93360), waypointClass(41.67574, -70.93295), waypointClass(41.67539, -70.93287)};

//#define NUMBER_WAYPOINTS 8 
//1st Sail CBC    waypointClass(41.60858, -70.92984), waypointClass(41.60809, -70.92958),
//waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(41.60808, -70.92841), waypointClass(41.60809, -70.92720), waypointClass(41.60811, -70.92544), waypointClass(41.60821, -70.92337), waypointClass(41.60739, -70.92454), waypointClass(41.60732, -70.92652), waypointClass(41.60735, -70.92832), waypointClass(41.60803, -70.93116)};

#define NUMBER_WAYPOINTS 4 
//2st Sail CBC    
waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(41.60772, -70.93021), waypointClass(41.60776, -70.92888), waypointClass(41.60725, -70.92941),waypointClass(41.60783, -70.93151)};

//waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(41.67591, -70.93487), waypointClass(41.67569, -70.93485)};

// ############################################################################################################################################
