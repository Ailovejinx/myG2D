/*
	Author: Anh-Dzung Doan
*/
#pragma once
#include "..\dependencies\inc\natives.h"
#include "..\dependencies\inc\types.h"
#include "..\dependencies\inc\enums.h"
#include "..\dependencies\inc\main.h"
#include <string>
#include <fstream>
#include <vector>
#include <stdio.h>
#pragma comment(lib, "gdiplus.lib")
#include <atlimage.h>

#define WIDEN2(x) L ## x
#define WIDEN(x) WIDEN2(x)
#define __WFILE__ WIDEN(__FILE__)
#define HRCHECK(__expr) {hr=(__expr);if(FAILED(hr)){wprintf(L"FAILURE 0x%08X (%i)\n\tline: %u file: '%s'\n\texpr: '" WIDEN(#__expr) L"'\n",hr, hr, __LINE__,__WFILE__);goto cleanup;}}
#define RELEASE(__p) {if(__p!=nullptr){__p->Release();__p=nullptr;}}

#define PI 3.14159265358979323846
#define DIRECTX 1
#define GDI 2
#define GTA_MAP 0
#define PROTAGONIST 1

//#define CAM_FOV 50


/*********************************************************************/
/******************************** STRUCTS **************************/
/*********************************************************************/

struct Quartenion {
	float x = -1, y = -1, z = -1, w = -1;
};

// an element in trajectory
struct Point {
	Vector3 player_coord;
	Vector3 cam_rot;
	Vector3 cam_coord;

};

// struct stores all necessary variables for GDI method
struct GDIScreenCaptureWorker
{
	int nScreenWidth;
	int nScreenHeight;
	HWND hDesktopWnd;
	HDC hDesktopDC;
	HDC hCaptureDC;
	HBITMAP hCaptureBitmap;

};


/*********************************************************************/
/******************************** FUNCTIONS **************************/
/*********************************************************************/

/******************************** Functions for screen capture ********************************/
void GDIInitScreenCapture(); // initialize screen capture object
void GDIReleaseScreenCapture(); // release screen capture object
bool GDITakeScreenshots(std::string file_name); // capture screen

/******************************** Functions for drawing menu ********************************/
void drawMenuLine(std::string caption, float lineWidth, float lineHeight, float lineTop,
	float lineLeft, float textLeft, bool active, bool title, bool rescaleText = true);
void drawRect(float A_0, float A_1, float A_2, float A_3, int A_4, int A_5, int A_6, int A_7);

/******************************** Functions for listening keyboard ********************************/
bool switchPressed(); // listen if users open main menu
void getButtonState(bool *select, bool *back, bool *up, bool *down); // listen keyboard buttons

/******************************** Functions that process menu ********************************/
void handleMainMenu(); // display main menu
void handleCreateTrajectoryMenu(std::string menu_name); // display menu of trajectory construction
void handleExecuteTrajectoryMenu(std::string menu_name); // display menu of trajectory execution

/******************************** Functions that create trajectory ********************************/
void addVertex(); // add vertex to file
bool getCoordsFromMarker(Vector3 &coords); // get coordinate (x,y,z=1) from the marker on the built-in map

/******************************** Functions that execute trajectory ********************************/
bool readyExecuteSparseTrajectory(); // check if we are ready to execute sparse trajectory
bool readyExecuteDenseTrajectory(); // check if we are ready to execute dense trajectory
void moveToStartingPoint(); // move the player to the first point within the sparse trajectory
void resetExecuteTrajectory(); // reset all parameters relating to functions of execute sparse/dense trajectory
void executeDenseTrajectory(); // simultaneously execute dense trajectory and collect images
void executeSparseTrajectory(); // execute sparse trajectory with/without creating dense trajectory
void setParametersOfExecuteSparseTrajectory(bool create_dense_trajectory); // set parameters for functions of execute sparse trajectory
void setParametersOfExecuteDenseTrajectory(); // set parameters for functions of execute dense trajectory

/******************************** Functions that deal with camera ********************************/
void createCamera(); // create our own camera
void updateCamera(float coord_x, float coord_y, float coord_z
	, float rot_x, float rot_y, float rot_z); // update 6D-pose to our own camera
void activateCamera(); // set our own camera rendering
void backToGameplayCamera(); // set default gameplay camera rendering

/******************************** Functions that display notifications ********************************/
void setNotificationText(std::string str, DWORD time = 1500, bool isGxtEntry = false); // set notification text
void updateNotificationText(); // set text style, position, font

/******************************** Functions to manipulate file stream ********************************/
bool isFileExist(std::string file_text); // check if file exists
bool readSparseTrajectory();
bool readDenseTrajectory();
bool readFirstPointInTrajectory();

/******************************** Other functions ****************************************************/
int updateFeatures(); // Update neccessary features within game, should be called in every frames
float computeDistanceXY(Vector3 a, Vector3 b); // Compute Euclidean distance, only take into account X and Y values
void ScriptMain(); // Main functions

/******************************** JYF's functions ****************************************************/

BOOL isOccluded(Vehicle vehicle, Vector3 cam_coord, Vector3 veh_coord); // 判断是否存在遮挡
void get_2DBB(Vehicle vehicle, Point cam, float* _xmin, float* _ymin, float* _xmax, float* _ymax); // 生成2DBB
void get_2D_from_3D(Point cam, Vector3 v, float* x2d, float* y2d);	//3D到2D图像上的坐标转换

void get_2D_from_3D_1(Vector3 world_coord, Point cam, float* x2D_pixel, float* y2D_pixel);
void get_angles(Point cam, Vector3 world_coord, Vehicle vehicle, float* alpha, float* r_y);
void get_vehicle_values(Vehicle vehicle, Vector3* upVector, Vector3* rightVector, Vector3* forwardVector, Vector3* position, Vector3* dim);
void get_vehicle_dim(Vehicle vehicle, Vector3* dim);
void world2cam(Point cam, Vector3 v, Vector3* d);
void get_R_matrix(Point cam, float R[9]);
void get_focal_length(int screenHeight, float* f);
