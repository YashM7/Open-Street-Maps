/*main.cpp*/

//
// Prof. Joe Hummel
// U. of Illinois, Chicago
// CS 251: Spring 2020
// Project #07: open street maps, graphs, and Dijkstra's alg
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:  
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <utility>
#include <set>
#include <limits>
#include <stack>
#include "tinyxml2.h"
#include "dist.h"
#include "osm.h"
#include "graph.h"

using namespace std;
using namespace tinyxml2;


class prioritize
{
   public:
   bool operator()(const pair<long long,double>& p1, const pair<long long,double>& p2) const
   {
      if(p1.second>p2.second)
         return true;
      else if(p1.second == p2.second)
      {   
         if(p1.first>p2.first)
         {
            return true;
         }
         else{
               return false;
             }
         }
      else
         return false;
      }
   };

//
// A function to check whether the building exists or not
// the function looks by Abbrevation and also by sub-string
//
bool BuildingCheck(string building, vector<BuildingInfo> &Buildings, BuildingInfo &object)
{
    for( auto &it : Buildings)
    {
        if(it.Abbrev == building)
        {
            object = it;
            return true;
        }
    }
    
    for( auto &it : Buildings)
    {
        if(it.Fullname.find(building) != string::npos)
        {
            object = it;
            return true;
        }
    }
    return false;
}


//
// A function to find the nearest start node from the start building and can also be used
// to find the nearest end node from the destination building
//
Coordinates NearestNode(double lat, double lon, vector<FootwayInfo> &Footways, map<long long, Coordinates> &Nodes)
{
    Coordinates coordinates;
    double minimun = 0.0;
    int first = 0;
    for(auto &it : Footways)
    {
        for(auto &IT : it.Nodes)
        {
            double distance = distBetween2Points(lat, lon, Nodes.at(IT).Lat, Nodes.at(IT).Lon);
            if(first == 0 || minimun > distance)
            {
                minimun = distance;
                coordinates = Nodes.at(IT);
            }
            first++;
        }
    }
    return coordinates;
}


//
// Dijkstra's algorithm to find the shortest path between the building
// using predecessor method
//
void Dijkstra(long long startV, map<long long, long long> &Previous, map<long long, double> &Distance, graph<long long, double> &G)
{
  const double INF = numeric_limits<double>::max();
  vector<long long>  visited;
  set<long long> visitedSet;
  vector<long long> V;
  
  priority_queue<
  pair<long long, double>,
  vector<pair<long long, double>>,
  prioritize> pq;
  
  V = G.getVertices();
  
  for(auto &it : V)
  {
     Distance[it] = INF;
     pq.push(make_pair(it,INF));
     Previous[it] = -1;
  }
   
   Distance[startV] = 0;
   Previous[startV] = 0;
   pq.push(make_pair(startV,0));
   
   while(!pq.empty())
  {
     long long currentV = pq.top().first;
     pq.pop(); 
        
     if(Distance[currentV] == INF)
     {
         break;
     }
     else if(visitedSet.count(currentV) == 1)
     {
         continue;
     }
     else
     {
        visitedSet.insert(currentV);
     }
     
     set<long long> Neighbour = G.neighbors(currentV);
     
     for(auto &IT : Neighbour)
     {   
        double edgeWeight = 0;
        G.getWeight(currentV, IT, edgeWeight);
        double alternativePathDistance = Distance[currentV] + edgeWeight;
        
        if(alternativePathDistance < Distance[IT]){
           Distance[IT] = alternativePathDistance;
           Previous[IT] = currentV;
           pq.push(make_pair(IT,alternativePathDistance));       
           }
        }
     }
}


//////////////////////////////////////////////////////////////////
//
// main
//
int main()
{
  map<long long, Coordinates>  Nodes;     // maps a Node ID to it's coordinates (lat, lon)
  vector<FootwayInfo>          Footways;  // info about each footway, in no particular order
  vector<BuildingInfo>         Buildings; // info about each building, in no particular order
  XMLDocument                  xmldoc;
  
  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "")
  {
    filename = def_filename;
  }

  //
  // Load XML-based map file 
  //
  if (!LoadOpenStreetMap(filename, xmldoc))
  {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }
  
  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == Nodes.size());
  assert(footwayCount == Footways.size());
  assert(buildingCount == Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  //
  // Build the graph, output stats:
  //
  graph<long long, double> G;
    
    
  //  
  // Adding nodes as vertices
  //   
  for(auto  &it : Nodes)
  {
      G.addVertex(it.first);
  }
    
    
  //
  // Add edges based on footway
  //
  for(auto &it : Footways)
  {
      for(size_t j = 0 ; j < it.Nodes.size() - 1 ; j++)
      {
          G.addEdge(it.Nodes[j], it.Nodes[j+1], distBetween2Points(Nodes.at(it.Nodes[j]).Lat, Nodes.at(it.Nodes[j]).Lon, Nodes.at(it.Nodes[j+1]).Lat, Nodes.at(it.Nodes[j+1]).Lon));
          G.addEdge(it.Nodes[j+1], it.Nodes[j], distBetween2Points(Nodes.at(it.Nodes[j]).Lat, Nodes.at(it.Nodes[j]).Lon, Nodes.at(it.Nodes[j+1]).Lat, Nodes.at(it.Nodes[j+1]).Lon));
      }
  }


  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;
    

  //
  // Navigation from building to building
  //
  string startBuilding, destBuilding;

  cout << "Enter start (partial name or abbreviation), or #> ";
  getline(cin, startBuilding);

  while (startBuilding != "#")
  {
    cout << "Enter destination (partial name or abbreviation)> ";
    getline(cin, destBuilding);

    //
    // lookup buildings, find nearest start and dest nodes,
    // run Dijkstra's alg, output distance and path to destination:
    //
    BuildingInfo start;
    BuildingInfo destination;
    bool StartBuilding = BuildingCheck(startBuilding, Buildings, start);
    bool DestinationBuilding = BuildingCheck(destBuilding, Buildings, destination);
      
    if(StartBuilding == false)
    {
        cout << "Start building not found" << endl;
        cout << "Enter start (partial name or abbreviation), or #> ";
        getline(cin, startBuilding);
        continue;
    }
    
    if(DestinationBuilding == false)
    {
        cout << "Destination building not found" << endl;
        cout << "Enter start (partial name or abbreviation), or #> ";
        getline(cin, startBuilding);
        continue;
    }
      
    if(StartBuilding == true)
    {
        cout << "Starting point:" << endl;
        cout << " " << start.Fullname << endl;
        cout << " " << "(" << start.Coords.Lat << ", " << start.Coords.Lon << ")" << endl;
    }

      
    if(DestinationBuilding == true)
    {
        cout << "Destination point:" << endl;
        cout << " " << destination.Fullname << endl;
        cout << " " << "(" << destination.Coords.Lat << ", " << destination.Coords.Lon << ")" << endl;
    }
    
    cout << endl;
    Coordinates StartCoords =  NearestNode(start.Coords.Lat, start.Coords.Lon, Footways, Nodes);
    Coordinates DestinationCoords = NearestNode(destination.Coords.Lat, destination.Coords.Lon, Footways, Nodes);
    cout << "Nearest start node:" << endl;
    cout << " " << StartCoords.ID << endl;
    cout << " " << "(" << StartCoords.Lat << ", " << StartCoords.Lon << ")" << endl;
      
    cout << "Nearest destination node:" << endl;
    cout << " " << DestinationCoords.ID << endl;
    cout << " " << "(" << DestinationCoords.Lat << ", " << DestinationCoords.Lon << ")" << endl;
      
    map<long long, long long> Previous;
    map<long long, double> Distance;
      
    Dijkstra(StartCoords.ID, Previous, Distance, G);
      
    stack<long long> path;
    long long a = DestinationCoords.ID;
      
    cout << endl;  
    cout << "Navigating with Dijkstra..." << endl;  
    if(Previous[a] == -1)
    {
        cout<<"Sorry, destination unreachable" << endl << endl;
        cout << "Enter start (partial name or abbreviation), or #> ";
        getline(cin, startBuilding);
        continue;
    }
      
    while(a != StartCoords.ID)
    {
        path.push(a);
        a = Previous[a];        
    }
    
    cout<<"Distance to dest: " << Distance[DestinationCoords.ID] << " miles" << endl;  
    cout << "Path: " << StartCoords.ID;       
    while(!path.empty())
    {
        cout << "->" << path.top();
        path.pop();
    }
      
    

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter start (partial name or abbreviation), or #> ";
    getline(cin, startBuilding);
  }

  //
  // done:
  //
  cout << "** Done **" << endl;

  return 0;
}