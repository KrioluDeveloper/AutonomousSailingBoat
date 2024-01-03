class  waypointClass
{
    
  public:
    waypointClass(double pLong = 0, double pLat = 0)
      {
        fLong = pLong;
        fLat = pLat;
      }
      
    double getLat(void) {return fLat;}
    double getLong(void) {return fLong;}

  private:
    double fLong, fLat;
      
  
};  // waypointClass


// usage as array: 
//waypointClass myWaypoints[4] = {waypointClass(1,2), waypointClass(2,3), waypointClass(4,5), waypointClass() };
