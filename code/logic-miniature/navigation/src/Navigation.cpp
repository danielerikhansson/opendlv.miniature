/**
 * Copyright (C) 2016 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <cstdlib>
#include <iostream>
#include <cmath>
#include <limits>
#include <cfloat>
#include <numeric>


#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <opendavinci/odcore/wrapper/Eigen.h>

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Navigation.h"

namespace opendlv {
namespace logic {
namespace miniature {



/*
  Constructor.
*/
Navigation::Navigation(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "logic-miniature-navigation")
    , m_mutex()
    , m_outerWalls()
    , m_innerWalls()
    , m_pointsOfInterest()
    , m_analogReadings()
    , m_gpioReadings()
    , m_gpioOutputPins()
    , m_pwmOutputPins()
    , arenaWidth()
    , arenaHeight()
    , arenaOffset()
    , gridCellSize(2) //in decimeters?
    , nbrGridCols()
    , nbrGridRows()
    , nbrGridCells()
    , m_sonarDistance()
    , asPath()
    , pwmValueLeftWheel()
    , pwmValueRightWheel()
    , m_positionX()
    , m_positionY()
    , m_positionYaw()
    , m_blinkLED()
    , FSMstate()
{
}

/*
  Destructor.
*/
Navigation::~Navigation()
{
}

/*
  This method reads values from the configuration file. Note that there is only
  one global configuration storage loaded by the central odsupercomponent
  module. If the the configuration file is changed, the odsupercompnent module
  needs to be restarted.
*/
void Navigation::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  std::string const gpioPinsString =
      kv.getValue<std::string>("logic-miniature-navigation.gpio-pins");
  std::vector<std::string> gpioPinsVector =
      odcore::strings::StringToolbox::split(gpioPinsString, ',');
  for (auto pin : gpioPinsVector) {
    m_gpioOutputPins.push_back(std::stoi(pin));
  }

  std::string const pwmPinsString =
      kv.getValue<std::string>("logic-miniature-navigation.pwm-pins");
  std::vector<std::string> pwmPinsVector =
      odcore::strings::StringToolbox::split(pwmPinsString, ',');
  for (auto pin : pwmPinsVector) {
    m_pwmOutputPins.push_back(std::stoi(pin));
  }

  std::string const outerWallsString =
      kv.getValue<std::string>("logic-miniature-navigation.outer-walls");
  std::vector<data::environment::Point3> outerWallPoints = ReadPointString(outerWallsString);
  if (outerWallPoints.size() == 4) {
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[0], outerWallPoints[1]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[1], outerWallPoints[2]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[2], outerWallPoints[3]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[3], outerWallPoints[0]));

    std::cout << "Outer walls 1 - " << m_outerWalls[0].toString() <<  std::endl;
    std::cout << "Outer walls 2 - " << m_outerWalls[1].toString() <<  std::endl;
    std::cout << "Outer walls 3 - " << m_outerWalls[2].toString() <<  std::endl;
    std::cout << "Outer walls 4 - " << m_outerWalls[3].toString() <<  std::endl;
  } else {
    std::cout << "Warning: Outer walls format error. (" << outerWallsString << ")" << std::endl;
  }

  std::string const innerWallsString =
      kv.getValue<std::string>("logic-miniature-navigation.inner-walls");
  std::vector<data::environment::Point3> innerWallPoints = ReadPointString(innerWallsString);
  for (uint32_t i = 0; i < innerWallPoints.size(); i += 2) {
    if (i < innerWallPoints.size() - 1) {
      data::environment::Line innerWall(innerWallPoints[i], innerWallPoints[i+1]);
      m_innerWalls.push_back(innerWall);
      std::cout << "Inner wall - " << innerWall.toString() << std::endl;
    }
  }

  std::string const pointsOfInterestString =
      kv.getValue<std::string>("logic-miniature-navigation.points-of-interest");
  m_pointsOfInterest = ReadPointString(pointsOfInterestString);
  for (uint32_t i = 0; i < m_pointsOfInterest.size(); i++) {
    std::cout << "Point of interest " << i << ": " << m_pointsOfInterest[i].toString() << std::endl;
  }

  // arena placement, account for arena being tilted!!!
  double minX = m_outerWalls[0].getA().getX();
  double minY = m_outerWalls[0].getA().getY();
  double maxX = m_outerWalls[0].getA().getX();
  double maxY = m_outerWalls[0].getA().getY();
  for (uint32_t i = 0; i < 4; i++) {
    if (m_outerWalls[i].getA().getX() < minX) {
      minX = m_outerWalls[i].getA().getX();
    }
    else if (m_outerWalls[i].getA().getX() > maxX) {
      maxX = m_outerWalls[i].getA().getX();
    }
    if (m_outerWalls[i].getB().getX() < minX) {
      minX = m_outerWalls[i].getB().getX();
    }
    else if (m_outerWalls[i].getB().getX() > maxX) {
      maxX = m_outerWalls[i].getB().getX();
    }
    if (m_outerWalls[i].getA().getY() < minY) {
      minY = m_outerWalls[i].getA().getY();
    }
    else  if (m_outerWalls[i].getA().getY() > maxY) {
      maxY = m_outerWalls[i].getA().getY();
    }
    if (m_outerWalls[i].getB().getY() < minY) {
      minY = m_outerWalls[i].getB().getY();
    }
    else if (m_outerWalls[i].getB().getY() > maxY) {
      maxY = m_outerWalls[i].getB().getY();
    }
  }
  arenaOffset.setX(minX);
  arenaOffset.setY(minY);
  arenaWidth = maxX-minX;
  arenaHeight = maxY-minY;
  nbrGridCols = round(arenaWidth/gridCellSize);
  nbrGridRows = round(arenaHeight/gridCellSize);
  nbrGridCells = nbrGridCols*nbrGridRows;
  std::cout << "Arena offset: " << arenaOffset.toString() << std::endl;
  std::cout << "Arena width: " << arenaWidth << std::endl;
  std::cout << "Arena height:  " << arenaHeight << std::endl;

}

/*
  This method is run automatically when the system is shutting down (before the
  destructor). It is typically used to close log files and de-allocate
  dynamically allocated memory.
*/
void Navigation::tearDown()
{
}




/*
  The while loop in this method runs at a predefined (in configuration)
  frequency.
*/
odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Navigation::body()
{
  FSMstate = "initialRotate";
  pwmValueLeftWheel = 35000;
  pwmValueRightWheel = 33500;
  uint32_t pwmValueServo = 1650000;
  int leftForward = 1;
  int rightForward = 1;
//  int counter = 0;
  //int blinkLED = 0;
  m_blinkLED = 0;
  opendlv::proxy::ToggleRequest::ToggleState leftRotation_1;
  opendlv::proxy::ToggleRequest::ToggleState leftRotation_2;
  opendlv::proxy::ToggleRequest::ToggleState rightRotation_1;
  opendlv::proxy::ToggleRequest::ToggleState rightRotation_2;
  opendlv::proxy::ToggleRequest::ToggleState blinkLED_state;

  int currentTarget = 0;

  data::environment::Point3 start = data::environment::Point3(0, 0, 0);
  //data::environment::Point3 randTarget = cellToPos(200);
  //data::environment::Point3 start = m_pointsOfInterest[1];
  data::environment::Point3 target = m_pointsOfInterest[currentTarget];


  std::cout << "Start point: " << start.toString() << "End point: " << target.toString() << std::endl;


  asPath = findPath(start, target);
  pathToString(asPath);


  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    // The mutex is required since 'body' and 'nextContainer' competes by
    // reading and writing to the class global maps, see also 'nextContainer'.
    odcore::base::Lock l(m_mutex);



    // Print some data collected from the 'nextContainer' method below.
    float voltageReadingPin0 = m_analogReadings[0];
    std::cout << "Reading from analog pin 0: " << voltageReadingPin0 << std::endl;

    int leftWhiskerActive = 0; // m_gpioReadings[26];
    int rightWhiskerActive = 0; //m_gpioReadings[46];

    std::cout << "leftWhiskerActive: " << leftWhiskerActive << std::endl;
    std::cout << "rightWhiskerActive: " << rightWhiskerActive << std::endl;

    std::cout << "State: " << FSMstate << std::endl;
//    std::cout << "Counter: " << counter << std::endl;


    // default values for wander base state
  // pwmValueLeftWheel = 35000;
  //  pwmValueRightWheel = 33500;



    pwmValueServo = 1650000;
    leftForward = 1;
    rightForward = 1;
    //blinkLED = 0;



    if (FSMstate == "initialRotate") {
      initialRotate();

    } else if (FSMstate == "pathFollowing") {
      updateWheelSpeeds_2(asPath);

    } else if (FSMstate == "targetReached") {
      currentTarget++;

      if ( currentTarget > 3){
        currentTarget = 0;
      }

      start = data::environment::Point3(m_positionX, m_positionY,0);
      target = m_pointsOfInterest[currentTarget];
      asPath = findPath(start, target);
      pathToString(asPath);

      FSMstate = "initialRotate";
    }












/*    if (leftWhiskerActive && rightWhiskerActive && FSMstate!="backUp" && FSMstate!="rotate") {
        FSMstate = "backUp";
    }
    else if (leftWhiskerActive && FSMstate!="backUp" && FSMstate!="rotate") {
     // pwmValueRightWheel = 10000;
	    FSMstate = "backUp";
    }
    else if (rightWhiskerActive && FSMstate!="backUp" && FSMstate!="rotate") {
     // pwmValueLeftWheel = 10000;
	    FSMstate = "backUp";
    }

    if (FSMstate == "backUp") {
      pwmValueServo = 2200000;
      //blinkLED = 1;
      leftForward = 0;
      rightForward = 0;
      if (counter > 20) {
        FSMstate = "rotate";
        counter = 0;
      }
      counter++;
    }
    if (FSMstate == "rotate") {
        std::cout << "################################: " << std::endl;
        std::cout << "################################: " << std::endl;
        std::cout << "################################: " << std::endl;
        std::cout << "################################: " << std::endl;
        std::cout << "################################: " << std::endl;
        std::cout << "################################: " << std::endl;
        pwmValueServo = 1100000;
        leftForward = 0;
        rightForward = 1;
        if (counter > 15) {
            FSMstate = "wander";
            counter = 0;
        }
        counter++;
    } */



    if (leftForward) {
      //leftRotation_1 = opendlv::proxy::ToggleRequest::Off; // real life
      //leftRotation_2 = opendlv::proxy::ToggleRequest::On; // Real life
      leftRotation_1 = opendlv::proxy::ToggleRequest::On; // Simulation
      leftRotation_2 = opendlv::proxy::ToggleRequest::Off; // Simulation

    } else {
      leftRotation_1 = opendlv::proxy::ToggleRequest::On; // real life
      leftRotation_2 = opendlv::proxy::ToggleRequest::Off; // real life
      leftRotation_1 = opendlv::proxy::ToggleRequest::Off; // Simulation
      leftRotation_2 = opendlv::proxy::ToggleRequest::On; // Simulation
    }

    if (rightForward) {
      rightRotation_1 = opendlv::proxy::ToggleRequest::On;
      rightRotation_2 = opendlv::proxy::ToggleRequest::Off;
    } else {
      rightRotation_1 = opendlv::proxy::ToggleRequest::Off;
      rightRotation_2 = opendlv::proxy::ToggleRequest::On;
    }

    if (m_blinkLED) {
      blinkLED_state = opendlv::proxy::ToggleRequest::On;
    } else {
      blinkLED_state = opendlv::proxy::ToggleRequest::Off;
    }

    opendlv::proxy::ToggleRequest requestLED(47, blinkLED_state);
    odcore::data::Container cLED(requestLED);
    getConference().send(cLED);

// ROTATION.
    opendlv::proxy::ToggleRequest request30(30, rightRotation_1);
    odcore::data::Container c30(request30);
    getConference().send(c30);

    opendlv::proxy::ToggleRequest request31(31, rightRotation_2);
    odcore::data::Container c31(request31);
    getConference().send(c31);

    opendlv::proxy::ToggleRequest request60(60, leftRotation_1);
    odcore::data::Container c60(request60);
    getConference().send(c60);

    opendlv::proxy::ToggleRequest request51(51, leftRotation_2);
    odcore::data::Container c51(request51);
    getConference().send(c51);

// VELOCITY.
    opendlv::proxy::PwmRequest requestRightWheel(0, pwmValueRightWheel);
    odcore::data::Container cRightWheel(requestRightWheel);
    cRightWheel.setSenderStamp(1);
    getConference().send(cRightWheel);

    opendlv::proxy::PwmRequest requestLeftWheel(0, pwmValueLeftWheel);
    odcore::data::Container cLeftWheel(requestLeftWheel);
    cLeftWheel.setSenderStamp(2);
    getConference().send(cLeftWheel);

    opendlv::proxy::PwmRequest requestServo(0, pwmValueServo);
    odcore::data::Container cServo(requestServo);
    cServo.setSenderStamp(3);
    getConference().send(cServo);

      std::cout << "[" << getName() << "] Sending PwmRequest chip0 (Right): "
          << requestRightWheel.toString() << std::endl;

      std::cout << "[" << getName() << "] Sending PwmRequest chip2 (Left): "
          << requestLeftWheel.toString() << std::endl;

      std::cout << "[" << getName() << "] Sending PwmRequest chip4: "
          << requestServo.toString() << std::endl;

          std::cout << "[" << getName() << "] Receiving distance: " << m_sonarDistance << std::endl;




    // testing A*

    //data::environment::Point3 randStart = cellToPos(rand() % nbrGridCells);
    //data::environment::Point3 randTarget = cellToPos(rand() % nbrGridCells);
    //data::environment::Point3 randStart = cellToPos(100);
    //data::environment::Point3 randTarget = cellToPos(120);
    //std::cout << "Start point: " << randStart.toString() << "End point: " << randTarget.toString() << std::endl;


  //  asPath = findPath(randStart, randTarget);
  //  pathToString(asPath);


  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


void Navigation::pathToString(std::vector<int> path) {
  for(auto i:path){
    std::cout << " --> " << i;
  }
  std::cout << endl;

  for (int i = 0; i < nbrGridCells; i++){
      std::vector<int>::iterator it;

      it = find (path.begin(), path.end(), i);
      if (it != path.end()){
        std::cout << "#";
      } else if(isFree(i)) {
        std::cout << "0";
      } else {
        std::cout << "X";
      }

      if ( ((i+1) % nbrGridCols) == 0 ) {
        std::cout << endl;
      }



  }
}



/* A* searh method
In progress
*/
std::vector<int> Navigation::findPath(data::environment::Point3 startPos, data::environment::Point3 targetPos){

  int startCell = posToCell(startPos);
  int targetCell = posToCell(targetPos);

  std::cout<< "Searching for path from " << startPos.toString();
  std::cout<< " to " << targetPos.toString() << std::endl;
  std::cout << "Start cell: " << startCell << std::endl;
  std::cout << "Target cell: " << targetCell << std::endl;

  std::vector<int> inOpenSet(nbrGridCells, 0);
  inOpenSet[startCell] = 1;
  std::vector<int> inClosedSet(nbrGridCells, 0);
  std::vector<int> cameFrom(nbrGridCells, -1);
  std::vector<double> gScore(nbrGridCells, DBL_MAX);
  gScore[startCell] = 0;
  std::vector<double> fScore(nbrGridCells, DBL_MAX);
  fScore[startCell] = euclidianDistance(startCell, targetCell);

  std::cout << "Running A* algorithm";
  int sumOpen = 1;
  while (sumOpen>0) {
    std::cout << ".";
    // find cell with lowest fScore
    int currentCell = 0;
    double currentScore = DBL_MAX;
    for (int i = 0; i < nbrGridCells; i++) {
      int iScore = fScore[i];
      if (inOpenSet[i]==1 && iScore < currentScore) {
        currentCell = i;
        currentScore = iScore;
      }
    }
    /*// debug print statements
    std::cout << "Original cell: " << startCell << std::endl;
    std::cout << "Target cell: " << targetCell << std::endl;
    std::cout << "Current cell: " << currentCell << std::endl;
    std::cout << "NbrCells: " << nbrGridCells << std::endl;
    std::cout << "nbrGridCols: " << nbrGridCols << std::endl;
    */
    // brake out if target's reached
    if (currentCell==targetCell) break;

    inOpenSet[currentCell] = 0;
    inClosedSet[currentCell] = 1;

    // Add neighbours
    std::vector<int> neighbours;
    if (currentCell%nbrGridCols>0) neighbours.push_back(currentCell-1);
    if ((currentCell+1)%nbrGridCols>0) neighbours.push_back(currentCell+1);
    if ((currentCell-nbrGridCols)>0) neighbours.push_back(currentCell-nbrGridCols);
    if ((currentCell+nbrGridCols)<nbrGridCells) neighbours.push_back(currentCell+nbrGridCols);

    int iNeighbour;
    double tentativeScore = gScore[currentCell] + gridCellSize;
    for (int i=0; i<(int)neighbours.size(); i++) {
      iNeighbour = neighbours[i];
      if (inClosedSet[iNeighbour]) continue;
      if (!isFree(iNeighbour)) continue;
      if (inOpenSet[iNeighbour]==0) {
        inOpenSet[iNeighbour] = 1;
      }
      else if (tentativeScore >= gScore[iNeighbour]) {
        continue;
      }
      cameFrom[iNeighbour] = currentCell;
      gScore[iNeighbour] = tentativeScore;
      fScore[iNeighbour] = tentativeScore + euclidianDistance(iNeighbour, targetCell);
    }

    sumOpen = std::accumulate(inOpenSet.begin(), inOpenSet.end(), 0);
  }

  //std::cout << "Finished while-loop" << std::endl;


  int iCell = targetCell;
  std::vector<int> path;
  path.push_back(iCell);
  while (iCell != startCell) {
    iCell = cameFrom[iCell];
    path.push_back(iCell);
  }
  std::reverse(path.begin(), path.end());


  std::cout << "path found" << std::endl;
  for (auto i:path) {
    std::cout << cellToPos(i).toString() << " --> ";
  }
  std::cout << std::endl;



  return path;
}


// Helper functions for coordinate transformation
int Navigation::posToCell(data::environment::Point3 position){
  double x = position.getX() - arenaOffset.getX(); //adjust for placement of arena
  double y = position.getY() - arenaOffset.getY(); //adjust for placement of arena
  //std::cout<< x << std::endl;
  //std::cout<< y << std::endl;
  int bigChunk =  round(y/gridCellSize)*nbrGridCols;
  int smallChunk = round(x/gridCellSize);
  //std::cout << bigChunk << std::endl;
  //std::cout << smallChunk << std::endl;
  return bigChunk+smallChunk;
}

data::environment::Point3 Navigation::cellToPos(int cell){
  int yRow = floor(cell/nbrGridCols);
  double y = yRow*gridCellSize;
  double x = (cell%nbrGridCols)*gridCellSize;
  data::environment::Point3 coordinates;
  coordinates.setX(x+arenaOffset.getX()); //adjust for placement of arena
  coordinates.setY(y+arenaOffset.getY()); //adjust for placement of arena
  return coordinates;
}
// Helper functions for distance
double Navigation::euclidianDistance(data::environment::Point3 p1, data::environment::Point3 p2){
  double squaredDist = pow(p1.getX()-p2.getX(), 2) + pow(p1.getY()-p2.getY(), 2) + pow(p1.getZ()-p2.getZ(), 2);
  return sqrt(squaredDist);
}
double Navigation::euclidianDistance(int cell1, int cell2){
  data::environment::Point3 p1 = cellToPos(cell1);
  data::environment::Point3 p2 = cellToPos(cell2);
  return euclidianDistance(p1, p2);
}
float Navigation::minDistance(data::environment::Line lineSegment, data::environment::Point3 p) {
  // Return minimum distance between lineSegment and point p

  data::environment::Point3 a = lineSegment.getA();
  data::environment::Point3 b = lineSegment.getB();
  float l2 = euclidianDistance(a, b);
  l2 = l2*l2;
  double t = dotProduct(p - a, b - a) / l2;
  if (t<0) t=0;
  if (t>1) t=1;
  data::environment::Point3 projection =  (b - a)*t + a;  // Projection falls on the segment
  return euclidianDistance(p, projection);
}
float Navigation::dotProduct(data::environment::Point3 vec1, data::environment::Point3 vec2) {
  return vec1.getX()*vec2.getX() + vec1.getY()*vec2.getY() + vec1.getZ()*vec2.getZ();
}
data::environment::Point3 Navigation::crossProduct(data::environment::Point3 vec1, data::environment::Point3 vec2)
{
    double x = vec1.getY() * vec2.getZ() - vec1.getZ() * vec2.getY();
    double y = vec1.getZ() * vec2.getX() - vec1.getX() * vec2.getZ();
    double z = vec1.getX() * vec2.getY() - vec1.getY() * vec2.getX();

    return data::environment::Point3(x, y, z);
}
// Helper function for availabilty of cell
bool Navigation::isFree(int cell) {
  data::environment::Point3 cellPos = cellToPos(cell);
  for (int i=0; i<(int)m_innerWalls.size(); i++) {
    double distanceToWall = minDistance(m_innerWalls[i], cellPos);
    if (distanceToWall < gridCellSize*1.5) {
      //std::cout << "|";
      return false;
    }
  }
  for (int i=0; i<(int)m_innerWalls.size(); i++) {
    double distanceToWall = minDistance(m_innerWalls[i], cellPos);
    if (distanceToWall < gridCellSize*1.5) {
      //std::cout << "|";
      return false;
    }
  }
  return true;
}

void Navigation::updateWheelSpeeds_2(std::vector<int> path) {


  double PI = 3.14159265;

  data::environment::Point3 carPosition = data::environment::Point3(m_positionX,m_positionY,0);
  data::environment::Point3 headToPosition;
  double carHeading = m_positionYaw;

  int currentNodeIndex = getNearestCell(carPosition, path);

  bool reachedTarget = false;

  int lookahead = 2;
  int pathSize = path.size();
  if ( currentNodeIndex + lookahead < pathSize ) {
    headToPosition = cellToPos(path[currentNodeIndex + lookahead]) - carPosition;

  } else if ( currentNodeIndex + lookahead -1 < pathSize ) {
    headToPosition = cellToPos(path[currentNodeIndex + lookahead -1]) - carPosition;
  } else {
    reachedTarget = true;
    FSMstate = "targetReached";
  }


  if (!reachedTarget) {
    double proportionality = 1;
    double angleToPoint = atan2(headToPosition.getY(), headToPosition.getX());
    double relativeAngle = (carHeading - angleToPoint) * proportionality / PI;


    std::cout << "relative angle: " << relativeAngle << endl;

    double vL = (1 + relativeAngle) / 2;
    double vR = (1 - relativeAngle) / 2;


//    pwmValueLeftWheel = 30000 + 20000 * vL;
//    pwmValueRightWheel = 28500 + 20000 * vR;

    // FOR SIMULATION

    pwmValueLeftWheel = 20000 + 30000 * vL;
    pwmValueRightWheel = 20000 + 30000 * vR;

  } else {
    pwmValueLeftWheel = 0;
    pwmValueRightWheel = 0;
  }
}


/* void navigation::angleCalc(double angelRob, double angelPoint) {

  if ((angleRob > PI /2 && angleToPoint < -PI/2) || angleRob < -PI/2 && angleToPoint > PI/2 ) {
    atan2()


  }



}*/

void Navigation::initialRotate() {

  double PI = 3.14159265;

  data::environment::Point3 carPosition = data::environment::Point3(m_positionX,m_positionY,0);
  data::environment::Point3 headToPosition = cellToPos(asPath[1]) - carPosition;
  double carHeading = m_positionYaw;

  double proportionality = 1;
  double angleToPoint = atan2(headToPosition.getY(), headToPosition.getX());
  double relativeAngle = (carHeading - angleToPoint) * proportionality / PI;

  if (relativeAngle > PI/10) {
    pwmValueLeftWheel = 30000;
    pwmValueRightWheel = 0;
  } else if (relativeAngle < -PI/10) {
    pwmValueLeftWheel = 30000;
    pwmValueRightWheel = 0000;
  } else {
      FSMstate = "pathFollowing";
  }
}




/*
    Method that takes a path and returns wheel speeds to incrementally follow
    the path.
*/

//vhy void, why not return the wheel speeds
void Navigation::updateWheelSpeeds(std::vector<int> path)
{
    data::environment::Point3 carPosition = data::environment::Point3(m_positionX,m_positionY,0); // TEMP, should probably be member
    data::environment::Point3 carHeading = data::environment::Point3(cos(m_positionYaw),sin(m_positionYaw),0);; // TEMP, should probably be member

    int currentNodeIndex = getNearestCell(carPosition,path); // TRIM vector?

    //current nodeIndex can be the previously passed node (behind the robot)!
    data::environment::Point3 headToPosition = cellToPos(path[currentNodeIndex]) - carPosition;
    data::environment::Point3 headToTangent = data::environment::Point3(0,0,0);
    // Look ahead (if not possible find the goal)

    int lookahead = 2;
    int pathSize = path.size();
    if ( currentNodeIndex + lookahead < pathSize)
    {
        headToPosition = cellToPos(path[currentNodeIndex + lookahead]);
        if ( currentNodeIndex + lookahead + 1 < pathSize)
        {
            headToTangent = cellToPos(path[currentNodeIndex + lookahead + 1]) - headToPosition;
        }
    }
    else if ( currentNodeIndex + lookahead -1 < pathSize)
    {
        headToPosition = cellToPos(path[currentNodeIndex + lookahead - 1]);
    }
    else
    {
        // CHECK IF CLOSE ENOUGH TO CONSIDERED FINISHED, PERHAPS DO THIS SOMEWHERE ELSE
        // LOWER SPEED
        m_blinkLED = 1;

    }

    carHeading.normalize();
    headToPosition.normalize();

    double proportionality = 0.5;
    double error = crossProduct(carHeading, headToPosition).getZ() * proportionality;

    double vL = (1 - error) / 2;
    double vR = (1 + error) / 2;


//    pwmValueLeftWheel = 30000 + 20000 * vL;
//    pwmValueRightWheel = 28500 + 20000 * vR;

    // FOR SIMULATION

    pwmValueLeftWheel = 18500 + 40000 * vL;
    pwmValueRightWheel = 18500 + 40000 * vR;


}

/*
    Finds the index of nearst point in path to another path
*/
//why not simply return the point from the path that is closest?
int Navigation::getNearestCell(data::environment::Point3 point, std::vector<int> path)
{
    if (path.size() == 0) { // Should never happen!
        return 0;
    }

    int bestMatchIndex = 0;
    double bestMatchLength = euclidianDistance(point, cellToPos(path[0]));

    for (int i=1; i<(int)path.size(); i++)
    {
        double length = euclidianDistance(point, cellToPos(path[i]));
        if ( length < bestMatchLength)
        {
            bestMatchIndex = i;
            bestMatchLength = length;
        }
    }

    return bestMatchIndex;
}

/*
  This method receives messages from all other modules (in the same conference
  id, cid). Here, the messages AnalogReading and ToggleReading is received
  from the modules interfacing to the hardware.
*/
void Navigation::nextContainer(odcore::data::Container &a_c)
{
  odcore::base::Lock l(m_mutex);

  int32_t dataType = a_c.getDataType();
  if (dataType == opendlv::proxy::AnalogReading::ID()) {
    opendlv::proxy::AnalogReading reading =
        a_c.getData<opendlv::proxy::AnalogReading>();

    uint16_t pin = reading.getPin();
    float voltage = reading.getVoltage();

    m_analogReadings[pin] = voltage; // Save the input to the class global map.

    std::cout << "[" << getName() << "] Received an AnalogReading: "
        << reading.toString() << "." << std::endl;

  } else if (dataType == opendlv::proxy::ToggleReading::ID()) {
    opendlv::proxy::ToggleReading reading =
        a_c.getData<opendlv::proxy::ToggleReading>();

    uint16_t pin = reading.getPin();
    bool state;
    if (reading.getState() == opendlv::proxy::ToggleReading::On) {
      state = true;
    } else {
      state = false;
    }

    m_gpioReadings[pin] = state; // Save the state to the class global map.


    std::cout << "[" << getName() << "] Received a ToggleReading: "
        << reading.toString() << "." << std::endl;


  } else if (dataType == opendlv::proxy::ProximityReading::ID()) {
      opendlv::proxy::ProximityReading reading =
        a_c.getData<opendlv::proxy::ProximityReading>();

      double distance = reading.getProximity();

      m_sonarDistance = distance;

      std::cout << "[" << getName() << "] Received a ProximityReading: "
          << reading.toString() << "." << std::endl;
  } else if (dataType == opendlv::model::State::ID()) {
    opendlv::model::State state =
        a_c.getData<opendlv::model::State>();

    double positionX = static_cast<double>(state.getPosition().getX());
    double positionY = static_cast<double>(state.getPosition().getY());
    double yaw = static_cast<double>(state.getAngularDisplacement().getZ());

    m_positionX = positionX;
    m_positionY = positionY;
    m_positionYaw = yaw;

    std::cout << "[" << getName() << "] Received a State: position "
        << positionX << ", " << positionY << " yaw " << yaw << "." << std::endl;
  }
}

std::vector<data::environment::Point3> Navigation::ReadPointString(std::string const &a_pointsString) const
{
  std::vector<data::environment::Point3> points;
  std::vector<std::string> pointStringVector =
      odcore::strings::StringToolbox::split(a_pointsString, ';');
  for (auto pointString : pointStringVector) {
    std::vector<std::string> coordinateVector =
        odcore::strings::StringToolbox::split(pointString, ',');
    if (coordinateVector.size() == 2) {
      double x = std::stod(coordinateVector[0]);
      double y = std::stod(coordinateVector[1]);
      double z = 0.0;
      points.push_back(data::environment::Point3(x, y, z));
    }
  }
  return points;
}

}
}
}
