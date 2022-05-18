float CurLon = 0.000;   //  Get Current Longitude from GPS
float CurLat = 52.000;  //  Get Current Latitude from GPS
float r_CurLon;         //  Current Longitude will be converted to Radians
float r_CurLat;         //  Current Latitude will be converted to Radians
float Bearing = 45;     //  Bearing of travel - Get current bearing from Compass
float r_Bearing;        //  Current Bearing will be converted to Radians
float Distance = 10;    // km per update 
int Eradius = 6371;     // mean radius of the earth 

void setup(void) 
{ 
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  r_CurLon = radians(CurLon);
  r_CurLat = radians(CurLat);
  r_Bearing = radians(Bearing);
  float DestLat = asin(sin(r_CurLat)*cos(Distance/Eradius)+cos(r_CurLat)*sin(Distance/Eradius)*cos(r_Bearing));                             //  This should be current waypoint latitude
  float DestLon = r_CurLon + atan2(sin(r_Bearing)*sin(Distance/Eradius)*cos(r_CurLat),cos(Distance/Eradius)-sin(r_CurLat)*sin(DestLat));    //  This should be current waypoint longitude
  DestLon = (DestLon+3*PI)/(2*PI);
  int i = DestLon;
  DestLon = (DestLon - i) * (2*PI) - PI;  // normalise to -180..+180º 
  
//System.out.printf("starting at a Longitude of %f and a Latitude of %f ",CurLon,CurLat); 
  Serial.print("starting at a Longitude of ");
  Serial.print(CurLon,6);
  Serial.print(" and a Latitude of ");
  Serial.println(CurLat,6);
//System.out.printf("if we travel %f km on a bearing of %f degrees ",Distance,Bearing); 
  Serial.print("if we travel ");
  Serial.print(Distance,6);
  Serial.print(" km on a bearing of ");
  Serial.print(Bearing,6);
  Serial.println(" degrees");
//System.out.printf("we end up at Longitude of %f and a Latitude of %f ",degrees(DestLon),degrees(DestLat)); 
  Serial.print("we end up at a Longitude of ");
  Serial.print(degrees(DestLon),6);
  Serial.print(" and a Latitude of ");
  Serial.println(degrees(DestLat),6);
} 

void loop(void)
{
}
