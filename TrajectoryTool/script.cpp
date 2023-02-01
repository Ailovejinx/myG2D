/*
	Author: Anh-Dzung Doan
*/

#include "script.h"
#include "keyboard.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

#define DEBUG TRUE


/*********************************************************************/
/******************************** VARIABLES **************************/
/*********************************************************************/

// 时间因子，控制慢镜头录制
float timeFactor = 8.0;
// 最大距离，远处的车辆没有参考意义
float maxDistance = 100.0;


// notification text
std::string _notification_text;
DWORD _notification_text_draw_ticks_max;
bool _notification_text_gxt_entry;

// Variables for processing menu
float _MENU_LINE_WIDTH = 350; // menu width size
int _current_main_menu_index = 0; // current selection index for main menu
int _current_create_trajectory_menu_index = 0; // current selection index for menu of trajectory creator
int _current_execute_trajectory_menu_index = 0; // current selection index for menu of trajectory executor

// threshold for trajectory executor
float _DISTANCE_THRESHOLD = 1.5;
float _DISTANCE_THRESHOLD_SMALL = 0.5;

// flags for trajectory executor
bool _DO_FOLLOW_TRAJECTORY = false;
bool _DO_CREATE_DENSE_TRAJECTORY = false;
bool _DO_FOLLOW_DENSE_TRAJECTORY = false;

int _traj_idx = 0; // trajectory index used while executing trajectory
Cam _camera; // Our owned camera used for executing dense trajectory and collecting image data
int _order_rot = 2; // rotation order

// The output file names
std::string _sparse_trajectory_file_text = "trajectory_sparse.txt";
//std::string _vertex_file_text = "vertex.txt";
std::string _vertex_file_text = "trajectory_sparse.txt";	// 最后用到的是trajectory_sparse.txt，方便起见直接生成之，即省去自定义轨迹序列这一步
std::string _dense_trajectory_file_text = "trajectory_dense.txt";
std::string _dataset_dir = "dataset";
std::string _dataset_image_dir = _dataset_dir + "/" + "image_2";
std::string _6dpose_im_file_text = "6dpose_list.txt";	// 输出的标注文件
std::string _anno_file_text_dir = _dataset_dir + "/" + "label_2";	// label输出路径
std::int32_t _anno_file_text_id = 0;	// 初始化label文件的ID，从000000.txt开始
std::string _calib_file_dir = _dataset_dir + "/" + "calib";    // 输出的相机标定文件

std::ofstream _ofile; // stream to write output files
std::ofstream _ofile_calib; // stream to write camera calib files

std::vector<Point> _trajectory; // trajectory vector used to store the trajectory (including sparse and dense trajectory)

GDIScreenCaptureWorker _screen_capture_worker; // variable for screen capture

// 车辆类别
std::string vehicleType[23] = {
	"Compacts",
	"Sedans",
	"SUVs",
	"Coupes",
	"Muscle",
	"Sports_Classics",
	"Sports",
	"Super",
	"Motorcycles",
	"Off-road",
	"Industrial",
	"Utility",
	"Vans",
	"Cycles",
	"Boats",
	"Helicopters",
	"Planes",
	"Service",
	"Emergency",
	"Military",
	"Commercial",
	"Trains",
	"Open Wheel",
};


/****************************************************************************************/
/******************************** IMPLEMENTATIONS OF FUNCTIONS **************************/
/****************************************************************************************/
void setNotificationText(std::string str, DWORD time /*= 1500*/, bool isGxtEntry /*= false*/)
{
	_notification_text = str;
	_notification_text_draw_ticks_max = GetTickCount() + time;
	_notification_text_gxt_entry = isGxtEntry;
}

void drawRect(float A_0, float A_1, float A_2, float A_3, int A_4, int A_5, int A_6, int A_7)
{
	GRAPHICS::DRAW_RECT((A_0 + (A_2 * 0.5f)), (A_1 + (A_3 * 0.5f)), A_2, A_3, A_4, A_5, A_6, A_7);
}

void drawMenuLine(std::string caption, float line_width, float line_height, float line_top,
	float line_left, float text_left, bool active, bool title, bool rescale_text /*= true*/)
{
	// default values
	int text_col[4] = { 255, 255, 255, 255 },
		rect_col[4] = { 70, 95, 95, 255 };
	float text_scale = 0.35;
	int font = 0;

	// correcting values for active line
	if (active)
	{
		text_col[0] = 0;
		text_col[1] = 0;
		text_col[2] = 0;

		rect_col[0] = 218;
		rect_col[1] = 242;
		rect_col[2] = 216;

		if (rescale_text) text_scale = 0.40;
	}

	if (title)
	{
		rect_col[0] = 0;
		rect_col[1] = 0;
		rect_col[2] = 0;

		if (rescale_text) text_scale = 0.50;
		font = 1;
	}

	int screen_w, screen_h;
	GRAPHICS::GET_SCREEN_RESOLUTION(&screen_w, &screen_h);

	text_left += line_left;

	float line_width_scaled = line_width / (float)screen_w; // line width
	float line_top_scaled = line_top / (float)screen_h; // line top offset
	float text_left_scaled = text_left / (float)screen_w; // text left offset
	float line_height_scaled = line_height / (float)screen_h; // line height

	float line_left_scaled = line_left / (float)screen_w;

	// this is how it's done in original scripts

	// text upper part
	UI::SET_TEXT_FONT(font);
	UI::SET_TEXT_SCALE(0.0, text_scale);
	UI::SET_TEXT_COLOUR(text_col[0], text_col[1], text_col[2], text_col[3]);
	UI::SET_TEXT_CENTRE(0);
	UI::SET_TEXT_DROPSHADOW(0, 0, 0, 0, 0);
	UI::SET_TEXT_EDGE(0, 0, 0, 0, 0);
	UI::_SET_TEXT_ENTRY("STRING");
	UI::_ADD_TEXT_COMPONENT_STRING((LPSTR)caption.c_str());
	UI::_DRAW_TEXT(text_left_scaled, (((line_top_scaled + 0.00278f) + line_height_scaled) - 0.005f));

	// text lower part
	UI::SET_TEXT_FONT(font);
	UI::SET_TEXT_SCALE(0.0, text_scale);
	UI::SET_TEXT_COLOUR(text_col[0], text_col[1], text_col[2], text_col[3]);
	UI::SET_TEXT_CENTRE(0);
	UI::SET_TEXT_DROPSHADOW(0, 0, 0, 0, 0);
	UI::SET_TEXT_EDGE(0, 0, 0, 0, 0);
	UI::_SET_TEXT_GXT_ENTRY("STRING");
	UI::_ADD_TEXT_COMPONENT_STRING((LPSTR)caption.c_str());
	int num25 = UI::_0x9040DFB09BE75706(text_left_scaled, (((line_top_scaled + 0.00278f) + line_height_scaled) - 0.005f));

	// rect
	drawRect(line_left_scaled, line_top_scaled + (0.00278f),
		line_width_scaled, ((((float)(num25)*UI::_0xDB88A37483346780(text_scale, 0)) + (line_height_scaled * 2.0f)) + 0.005f),
		rect_col[0], rect_col[1], rect_col[2], rect_col[3]);
}

void GDIInitScreenCapture()
{
	_screen_capture_worker.nScreenWidth = GetSystemMetrics(SM_CXSCREEN);
	_screen_capture_worker.nScreenHeight = GetSystemMetrics(SM_CYSCREEN);
	_screen_capture_worker.hDesktopWnd = GetDesktopWindow();
	_screen_capture_worker.hDesktopDC = GetDC(_screen_capture_worker.hDesktopWnd);
	_screen_capture_worker.hCaptureDC = CreateCompatibleDC(_screen_capture_worker.hDesktopDC);

	_screen_capture_worker.hCaptureBitmap = CreateCompatibleBitmap(_screen_capture_worker.hDesktopDC,
		_screen_capture_worker.nScreenWidth, _screen_capture_worker.nScreenHeight);

	SelectObject(_screen_capture_worker.hCaptureDC, _screen_capture_worker.hCaptureBitmap);
}

void GDIReleaseScreenCapture()
{
	ReleaseDC(_screen_capture_worker.hDesktopWnd, _screen_capture_worker.hDesktopDC);
	DeleteDC(_screen_capture_worker.hCaptureDC);
	DeleteObject(_screen_capture_worker.hCaptureBitmap);
}

bool GDITakeScreenshots(std::string file_name)
{
	BitBlt(_screen_capture_worker.hCaptureDC, 0, 0,
		_screen_capture_worker.nScreenWidth, _screen_capture_worker.nScreenHeight,
		_screen_capture_worker.hDesktopDC, 0, 0, SRCCOPY | CAPTUREBLT);
	CImage image;
	image.Attach(_screen_capture_worker.hCaptureBitmap);
	image.Save(file_name.c_str(), Gdiplus::ImageFormatJPEG);

	return true;
}

bool switchPressed()
{
	return IsKeyJustUp(VK_F5);
}

void getButtonState(bool* select, bool* back, bool* up, bool* down)
{
	if (select) *select = IsKeyDown(VK_NUMPAD5);
	if (back) *back = IsKeyDown(VK_NUMPAD0) || switchPressed() || IsKeyDown(VK_BACK);
	if (up) *up = IsKeyDown(VK_NUMPAD8);
	if (down) *down = IsKeyDown(VK_NUMPAD2);
}

void addVertex(int vertex_src)
{

	Vector3 coord;
	bool success;
	if (vertex_src == GTA_MAP)
	{
		success = getCoordsFromMarker(coord); // get XYZ location from the marker on the built-in map
		if (!success)
		{
			setNotificationText("Please put the marker on the map");
			return;
		}
	}
	else
	{
		Entity player_ped = PLAYER::PLAYER_PED_ID();
		coord = ENTITY::GET_ENTITY_COORDS(player_ped, true);
		success = true;
	}

	std::ofstream ofile;

	// if vertex file does not exist, create a new one
	if (!isFileExist(_vertex_file_text))
	{
		ofile.open(_vertex_file_text.c_str(), std::ios_base::out);
	}

	// if vertex file exists, append the content
	else
	{
		ofile.open(_vertex_file_text.c_str(), std::ios_base::app);
	}

	ofile << coord.x << " " << coord.y << " " << coord.z << std::endl;
	setNotificationText("Add vertex: " + std::to_string(coord.x) + "," + std::to_string(coord.y) + "," + std::to_string(coord.z));
	ofile.close();
}

void handleCreateTrajectoryMenu(std::string menu_name)
{
	const int menu_item_number = 3;
	std::string menu_list[menu_item_number] = { "ADD VERTEX FROM MARKER ON THE MAP", "ADD VERTEX FROM PROTAGONIST POSITION", "..." };

	DWORD wait_time = 150;

	while (true)
	{
		// timed menu draw, used for pause after active line switch
		DWORD max_tick_count = GetTickCount() + wait_time;
		do
		{
			// draw menu
			drawMenuLine(menu_name, _MENU_LINE_WIDTH, 15.0, 18.0, 0.0, 5.0, false, true);
			for (int i = 0; i < menu_item_number; i++)
				if (i != _current_create_trajectory_menu_index)
					drawMenuLine(menu_list[i], _MENU_LINE_WIDTH, 9.0, 60.0 + i * 36.0, 0.0, 9.0, false, false);
			drawMenuLine(menu_list[_current_create_trajectory_menu_index],
				_MENU_LINE_WIDTH + 1.0, 11.0, 56.0 + _current_create_trajectory_menu_index * 36.0, 0.0, 7.0, true, false);
			updateFeatures();
			WAIT(0);
		} while (GetTickCount() < max_tick_count);

		wait_time = 0;

		// listen if users press any buttons
		bool button_select, button_back, button_up, button_down;
		getButtonState(&button_select, &button_back, &button_up, &button_down);

		if (button_select) // if select button is pressed
		{
			switch (_current_create_trajectory_menu_index)
			{
			case 0:
				addVertex(GTA_MAP);
				break;
			case 1:
				addVertex(PROTAGONIST);
				break;
			case 2:
				setNotificationText("Need your help for more convenient ways to create trajectory");
				break;
			}
			wait_time = 200;
		}
		else
			if (button_back || switchPressed()) // if back button is pressed
			{
				break;
			}
			else
				if (button_up) // if up/down button is pressed
				{

					if (_current_create_trajectory_menu_index == 0)
						_current_create_trajectory_menu_index = menu_item_number;
					_current_create_trajectory_menu_index--;
					wait_time = 150;
				}
				else
					if (button_down)
					{
						_current_create_trajectory_menu_index++;
						if (_current_create_trajectory_menu_index == menu_item_number)
							_current_create_trajectory_menu_index = 0;
						wait_time = 150;
					}
	}

}

bool readFirstPointInTrajectory()
{
	// if unable to load sparse trajectory, return false
	if (!isFileExist(_sparse_trajectory_file_text))
	{
		return false;
	}

	// only read the first point
	std::ifstream file(_sparse_trajectory_file_text.c_str());
	_trajectory.clear();
	Point p;
	file >> p.player_coord.x;
	file >> p.player_coord.y;
	file >> p.player_coord.z;

	_trajectory.push_back(p);
	return true;
}
bool readSparseTrajectory()
{
	// if unable to load sparse trajectory, return false
	if (!isFileExist(_sparse_trajectory_file_text))
	{
		return false;
	}

	// read all points within sparse trajectory
	std::ifstream file(_sparse_trajectory_file_text.c_str());
	_trajectory.clear();
	while (!file.eof())
	{
		Point p;
		file >> p.player_coord.x;
		file >> p.player_coord.y;
		file >> p.player_coord.z;

		_trajectory.push_back(p);
	}
	_trajectory.pop_back(); // last and second last elements are the same

	if (_trajectory.size() < 1)
		return false;

	return true;
}

bool readDenseTrajectory()
{
	// if unable to load dense trajectory, return false
	if (!isFileExist(_dense_trajectory_file_text))
	{
		return false;
	}

	// read all points in dense trajectory
	std::ifstream file(_dense_trajectory_file_text.c_str());
	_trajectory.clear();
	while (!file.eof())
	{
		Point p;
		int temp;

		// read player location
		file >> p.player_coord.x;
		file >> p.player_coord.y;
		file >> p.player_coord.z;

		// read 6d pose of camera
		file >> p.cam_coord.x;
		file >> p.cam_coord.y;
		file >> p.cam_coord.z;

		file >> p.cam_rot.x;
		file >> p.cam_rot.y;
		file >> p.cam_rot.z;

		file >> temp;
		file >> temp;
		_trajectory.push_back(p);
	}
	_trajectory.pop_back(); // last and second last elements are the same

	if (_trajectory.size() < 1)
		return false;

	return true;
}

void moveToStartingPoint()
{
	if (readFirstPointInTrajectory() == false)
	{
		setNotificationText("Unable to load sparse trajectory");
		return;
	}

	// if we do not know Z, we should find Z
	if (_trajectory[0].player_coord.z == 1)
	{
		Vector3 coords;
		coords.x = _trajectory[0].player_coord.x;
		coords.y = _trajectory[0].player_coord.y;
		coords.z = 1;
		bool groundFound = false;
		Entity e = PLAYER::PLAYER_PED_ID();
		static float groundCheckHeight[] = {
			100.0, 150.0, 50.0, 0.0, 200.0, 250.0, 300.0, 350.0, 400.0,
			450.0, 500.0, 550.0, 600.0, 650.0, 700.0, 750.0, 800.0
		};
		for (int i = 0; i < sizeof(groundCheckHeight) / sizeof(float); i++)
		{
			ENTITY::SET_ENTITY_COORDS_NO_OFFSET(e, coords.x, coords.y, groundCheckHeight[i], 0, 0, 1);
			WAIT(100);
			if (GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(coords.x, coords.y, groundCheckHeight[i], &coords.z, FALSE))
			{
				groundFound = true;
				coords.z += 1.0;
				break;
			}
		}

		// if ground not found then set Z in air and give player a parachute
		if (!groundFound)
		{
			coords.z = 1000.0;
			WEAPON::GIVE_DELAYED_WEAPON_TO_PED(PLAYER::PLAYER_PED_ID(), 0xFBAB5776, 1, 0);
			setNotificationText("Moved player to starting point with a parachute, it is not good, should find for other initial coordinate");
		}
		else
		{
			ENTITY::SET_ENTITY_COORDS_NO_OFFSET(e, coords.x, coords.y, coords.z, 0, 0, 1);
			WAIT(0);
			setNotificationText("Moved to starting point");
		}

	}
	else
	{
		Entity e = PLAYER::PLAYER_PED_ID();

		ENTITY::SET_ENTITY_COORDS_NO_OFFSET(e, _trajectory[0].player_coord.x, _trajectory[0].player_coord.y, _trajectory[0].player_coord.z, 0, 0, 1);

		WAIT(0);
		setNotificationText("Moved to starting point");
	}

}

void createCamera()
{
	_camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", 1); // create our own camera
	CAM::SET_CAM_FOV(_camera, CAM::GET_GAMEPLAY_CAM_FOV()); // set its fov
}

void updateCamera(float coord_x, float coord_y, float coord_z
	, float rot_x, float rot_y, float rot_z)
{
	CAM::SET_CAM_COORD(_camera, coord_x, coord_y, coord_z); // update location of our own camera
	CAM::SET_CAM_ROT(_camera, rot_x, rot_y, rot_z, _order_rot); // update rotation of our own camera
}

void activateCamera()
{
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE); // set our own camera rendering
}

void backToGameplayCamera()
{
	CAM::RENDER_SCRIPT_CAMS(false, 1, 1, 1, 0); // set gameplay camera rendering
}

bool readyExecuteSparseTrajectory()
{
	if (readSparseTrajectory() == false)
	{
		setNotificationText("Unable to load sparse trajectory");
		return false;
	}

	// get player location
	Entity player_ped = PLAYER::PLAYER_PED_ID();
	Vector3 player_coord = ENTITY::GET_ENTITY_COORDS(player_ped, true);

	bool traj_idx = 0;

	// check if player location is too far from the first point of sparse trajectory
	float dist = computeDistanceXY(player_coord, _trajectory[traj_idx].player_coord);
	if (dist > _DISTANCE_THRESHOLD_SMALL)
	{
		setNotificationText("Please move to starting point first");

		return false;
	}

	return true;
}

bool readyExecuteDenseTrajectory()
{
	if (readDenseTrajectory() == false)
	{
		setNotificationText("Unable to load dense trajectory");
		return false;
	}
	return true;
}

void executeSparseTrajectory()
{
	// We get player location to store in dense trajectory
	Entity player_ped = PLAYER::PLAYER_PED_ID();
	Vector3 player_coord = ENTITY::GET_ENTITY_COORDS(player_ped, true);

	// We get 6D pose of gameplay camera to store in dense trajectory
	Vector3 gameplay_cam_coord = CAM::GET_GAMEPLAY_CAM_COORD();
	Vector3 gameplay_cam_rot = CAM::GET_GAMEPLAY_CAM_ROT(_order_rot);

	// If trajectory is too small
	if (_trajectory.size() <= 1)
	{
		setNotificationText("Please add more one point in trajectory");
		resetExecuteTrajectory();
		return;
	}

	// if player reaches to the destination point, move to next point
	float dist = computeDistanceXY(player_coord, _trajectory[_traj_idx].player_coord);
	if (dist < _DISTANCE_THRESHOLD)
	{
		_traj_idx++;
		AI::TASK_GO_STRAIGHT_TO_COORD(player_ped, _trajectory[_traj_idx].player_coord.x, _trajectory[_traj_idx].player_coord.y, _trajectory[_traj_idx].player_coord.z,
			2, 60000, 1, 0);

		setNotificationText("Go to point " + std::to_string(_traj_idx + 1) +
			": (x,y) = " + "(" + std::to_string(_trajectory[_traj_idx].player_coord.x) + "," +
			std::to_string(_trajectory[_traj_idx].player_coord.y) + ")");
	}

	// Save dense trajectory 
	if (!_ofile.is_open()) // if dense trajectory file has not been opened, open it
		_ofile.open(_dense_trajectory_file_text.c_str());

	_ofile << player_coord.x << " " << player_coord.y << " " << player_coord.z << " ";
	_ofile << gameplay_cam_coord.x << " " << gameplay_cam_coord.y << " " << gameplay_cam_coord.z << " ";
	_ofile << gameplay_cam_rot.x << " " << gameplay_cam_rot.y << " " << gameplay_cam_rot.z << " ";
	_ofile << _traj_idx << " " << _traj_idx + 1 << std::endl;

	// if we reach to last point of trajectory, stop function
	if (_traj_idx == _trajectory.size() - 1)
	{
		resetExecuteTrajectory(); // reset all parameters

		if (_ofile.is_open())
			_ofile.close();

		return;
	}
}

// 生成带标注的数据集
void executeDenseTrajectory()
{
	Player mainPlayer = PLAYER::PLAYER_ID();
	// 将玩家设为无敌且被所有人忽略，防止被车撞后采集终止
	PLAYER::SET_PLAYER_INVINCIBLE(mainPlayer, TRUE);
	PLAYER::SET_EVERYONE_IGNORE_PLAYER(mainPlayer, TRUE);
	PLAYER::SET_POLICE_IGNORE_PLAYER(mainPlayer, TRUE);
	// 清除悬赏等级
	PLAYER::CLEAR_PLAYER_WANTED_LEVEL(mainPlayer);

	// 慢动作
	GAMEPLAY::SET_TIME_SCALE(1.0f / timeFactor);

	if (_trajectory.size() <= 1)
	{
		setNotificationText("Please add more one point in trajectory");
		resetExecuteTrajectory();
		return;
	}

	// if player reaches to the final point of dense trajectory, we set gameplay camera rendering
	//_traj_idx就是生成图片的id，也是_anno_file_text_id
	_anno_file_text_id = _traj_idx;

	// 建立VEHICLE矩阵，获取所有车辆
	const int ARR_SIZE = 1024;
	Vehicle* vehicles = new Vehicle[ARR_SIZE];
	//Vehicle vehicles[ARR_SIZE];
	int count = worldGetAllVehicles(vehicles, ARR_SIZE);

	if (_traj_idx == _trajectory.size())
	{
		GAMEPLAY::SET_TIME_SCALE(1.0f);	// 这时数据采集结束，结束慢动作
		resetExecuteTrajectory();

		if (CAM::IS_CAM_RENDERING(_camera) == TRUE)
			backToGameplayCamera();

		if (_ofile.is_open())
			_ofile.close();
		return;
	}
	else
		// if first point of dense trajectory is the point destination
		// we teleport the player to that first point
		// because it needs time for game to render all game objects, we let our program wait a little bit
		if (_traj_idx == 0)
		{
			// teleport the player
			/*Entity e = PLAYER::PLAYER_PED_ID();*/
			ENTITY::SET_ENTITY_COORDS_NO_OFFSET(mainPlayer, _trajectory[0].player_coord.x, _trajectory[0].player_coord.y, _trajectory[0].player_coord.z, 0, 0, 1);

			WAIT(1000);

			// set 6D pose for our own camera
			updateCamera(_trajectory[_traj_idx].cam_coord.x, _trajectory[_traj_idx].cam_coord.y, _trajectory[_traj_idx].cam_coord.z,
				_trajectory[_traj_idx].cam_rot.x, _trajectory[_traj_idx].cam_rot.y, _trajectory[_traj_idx].cam_rot.z);

			//_ofile.open(_dataset_dir + "/" + _6dpose_im_file_text);

			// if our own camera is not the rendering camera, we set it rendering
			if (CAM::IS_CAM_RENDERING(_camera) == false)
				activateCamera();
			WAIT(1000); // wait for the game to update its objects

			// 获取文件名称
			std::stringstream ss, ss1;
			ss << std::setfill('0') << std::setw(6) << std::to_string(_anno_file_text_id) << ".txt";
			ss1 << std::setfill('0') << std::setw(6) << std::to_string(_anno_file_text_id) << ".png";

			std::string _anno_file_text_name = ss.str();
			std::string im_name = _dataset_image_dir + "/" + ss1.str();


			GDITakeScreenshots(im_name); // capture screen


			// 打开文件
			_ofile.open(_anno_file_text_dir + "/" + _anno_file_text_name);


			// store 6D pose
			_ofile << im_name << " " << std::to_string(_trajectory[_traj_idx].cam_coord.x) << " " <<
				std::to_string(_trajectory[_traj_idx].cam_coord.y) << " " << std::to_string(_trajectory[_traj_idx].cam_coord.z) << " " <<
				std::to_string(_trajectory[_traj_idx].cam_rot.x) << " " << std::to_string(_trajectory[_traj_idx].cam_rot.y) << " " <<
				std::to_string(_trajectory[_traj_idx].cam_rot.z) << std::endl;
			_ofile.close();


			WAIT(100);
		}
		else
		{

			// Let the protagonist move to get better graphics for collecting large-scale dataset
			/*Entity player_ped = PLAYER::PLAYER_PED_ID();*/
			ENTITY::SET_ENTITY_COORDS_NO_OFFSET(mainPlayer, _trajectory[_traj_idx].player_coord.x, _trajectory[_traj_idx].player_coord.y, _trajectory[_traj_idx].player_coord.z, 0, 0, 1);

			// set 6D pose for our own camera
			updateCamera(_trajectory[_traj_idx].cam_coord.x, _trajectory[_traj_idx].cam_coord.y, _trajectory[_traj_idx].cam_coord.z,
				_trajectory[_traj_idx].cam_rot.x, _trajectory[_traj_idx].cam_rot.y, _trajectory[_traj_idx].cam_rot.z);

			Point cam;

			cam.cam_coord = CAM::GET_CAM_COORD(_camera);
			cam.cam_rot = CAM::GET_CAM_ROT(_camera, 2);

			WAIT(200);

			// 获取文件名称
			std::stringstream ss, ss1;
			ss << std::setfill('0') << std::setw(6) << std::to_string(_anno_file_text_id) << ".txt";
			ss1 << std::setfill('0') << std::setw(6) << std::to_string(_anno_file_text_id) << ".png";

			std::string _anno_file_text_name = ss.str();
			std::string im_name = _dataset_image_dir + "/" + ss1.str();

			// capture screen
			GDITakeScreenshots(im_name);


			// 打开文件
			_ofile.open(_anno_file_text_dir + "/" + _anno_file_text_name);
			_ofile_calib.open(_calib_file_dir + "/" + _anno_file_text_name);
			

			for (int i = 0; i < count; ++i)
			{
				// 判断车辆是否在屏幕上，否 则跳过
				if (!ENTITY::IS_ENTITY_ON_SCREEN(vehicles[i])) {
					continue;
				}

				Vector3 veh_coords = ENTITY::GET_ENTITY_COORDS(vehicles[i], TRUE);	// 获取车辆坐标
				float veh2cam_distance = GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(
					cam.cam_coord.x, cam.cam_coord.y, cam.cam_coord.z,
					veh_coords.x, veh_coords.y, veh_coords.z, 1
				);	// 计算车辆到相机之间的距离

				// 不考虑远处车辆
				if (veh2cam_distance < maxDistance) {
					int veh_type = VEHICLE::GET_VEHICLE_CLASS(vehicles[i]);

					BOOL occluded = isOccluded(vehicles[i], cam.cam_coord, veh_coords);	// 获取遮挡信息

					// 获取2DBB
					float xmin, ymin, xmax, ymax;
					get_2DBB(vehicles[i], cam, &xmin, &ymin, &xmax, &ymax);

					// truncated
					BOOL flag1, flag2, flag3, flag4, truncated_flag;
					flag1 = xmin >= 0;
					flag2 = ymin >= 0;
					flag3 = xmax <= _screen_capture_worker.nScreenWidth;
					flag4 = ymax <= _screen_capture_worker.nScreenHeight;

					truncated_flag = !(flag1 && flag2 && flag3 && flag4);

					// angles
					float alpha, r_y;
					get_angles(cam, veh_coords, vehicles[i], &alpha, &r_y);

					// dim
					Vector3 dim;
					get_vehicle_dim(vehicles[i], &dim);

					// coord in cam coord system
					Vector3 veh_coords_cam;

					world2cam(cam, veh_coords, &veh_coords_cam);

					//// store annotation
					//_ofile << vehicleType[veh_type] << " " << veh2cam_distance << " " << truncated_flag << " " <<
					//	occluded << " " << "alpha" << " " <<
					//	xmin << " " << ymin << " " << xmax << " " << ymax << std::endl;

					// store as KITTI form
					_ofile << "Car" << " " << truncated_flag << " " <<
						occluded << " " << alpha << " " <<
						xmin << " " << ymin << " " << xmax << " " << ymax << " " <<
						dim.x << " " << dim.y << " " << dim.z << " " <<
						veh_coords_cam.x << " " << veh_coords_cam.y << " " << veh_coords_cam.z << " " << r_y << " " << std::endl;

					//store KITTI calib files
					_ofile_calib << "P2:" << " " << 
				}



			}
			// 释放存放车辆的数组
			delete[] vehicles;

			_ofile.close();
			_ofile_calib.close();



			WAIT(200);
		}

	_traj_idx++;
}

BOOL isOccluded(Vehicle vehicle, Vector3 cam_coord, Vector3 veh_coord)
{
	// 判断是否存在遮挡

	// judge if the cam to the vehicle is occluded by something
	// useful to detecting occlusions of vehicles
	Vector3 end_coords1, surface_norm1;
	BOOL occlusion_veh;
	Entity entityHit1 = 0;

	int ray_veh_occlusion = WORLDPROBE::_CAST_RAY_POINT_TO_POINT(
		cam_coord.x, cam_coord.y, cam_coord.z,
		veh_coord.x, veh_coord.y, veh_coord.z,
		(~0 ^ (8 | 4)), vehicle, 7
	);

	WORLDPROBE::_GET_RAYCAST_RESULT(ray_veh_occlusion, &occlusion_veh, &end_coords1, &surface_norm1, &entityHit1);

	return occlusion_veh;

}


void get_2DBB(Vehicle vehicle, Point cam, float* _xmin, float* _ymin, float* _xmax, float* _ymax)
{
	// 生成2DBB

	int screenWidth = _screen_capture_worker.nScreenWidth;
	int screenHeight = _screen_capture_worker.nScreenHeight;
	//setNotificationText(std::to_string(screenWidth) + "x" + std::to_string(screenHeight));

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Left
	Vector3 upVector, rightVector, forwardVector, position; //entity position
	Vector3 dim;

	get_vehicle_values(vehicle, &upVector, &rightVector, &forwardVector, &position, &dim);

	//calculate point FUR and BLL from the center coord
	FUR.x = position.x + dim.y * rightVector.x + dim.x * forwardVector.x + dim.z * upVector.x;
	FUR.y = position.y + dim.y * rightVector.y + dim.x * forwardVector.y + dim.z * upVector.y;
	FUR.z = position.z + dim.y * rightVector.z + dim.x * forwardVector.z + dim.z * upVector.z;

	BLL.x = position.x - dim.y * rightVector.x - dim.x * forwardVector.x - dim.z * upVector.x;
	BLL.y = position.y - dim.y * rightVector.y - dim.x * forwardVector.y - dim.z * upVector.y;
	BLL.z = position.z - dim.y * rightVector.z - dim.x * forwardVector.z - dim.z * upVector.z;

	//get_2D_from_3D(cam, FUR, &xmin, &ymin);
	//get_2D_from_3D(cam, BLL, &xmax, &ymax);

	// calculate 8 vertex of object
	Vector3 edge[8];

	edge[0] = BLL;
	edge[4] = FUR;


	edge[1].x = edge[0].x + 2 * dim.y * rightVector.x;
	edge[1].y = edge[0].y + 2 * dim.y * rightVector.y;
	edge[1].z = edge[0].z + 2 * dim.y * rightVector.z;

	edge[2].x = edge[1].x + 2 * dim.z * upVector.x;
	edge[2].y = edge[1].y + 2 * dim.z * upVector.y;
	edge[2].z = edge[1].z + 2 * dim.z * upVector.z;

	edge[3].x = edge[0].x + 2 * dim.z * upVector.x;
	edge[3].y = edge[0].y + 2 * dim.z * upVector.y;
	edge[3].z = edge[0].z + 2 * dim.z * upVector.z;

	edge[5].x = edge[4].x - 2 * dim.y * rightVector.x;
	edge[5].y = edge[4].y - 2 * dim.y * rightVector.y;
	edge[5].z = edge[4].z - 2 * dim.y * rightVector.z;

	edge[6].x = edge[5].x - 2 * dim.z * upVector.x;
	edge[6].y = edge[5].y - 2 * dim.z * upVector.y;
	edge[6].z = edge[5].z - 2 * dim.z * upVector.z;

	edge[7].x = edge[4].x - 2 * dim.z * upVector.x;
	edge[7].y = edge[4].y - 2 * dim.z * upVector.y;
	edge[7].z = edge[4].z - 2 * dim.z * upVector.z;

	Vector3 edge2D[8];
	float x2D[8], y2D[8];

	for (int i = 0; i < 8; ++i) {
		//get_2D_from_3D_1(edge[i], cam, &edge2D[i].x, &edge2D[i].y);
		get_2D_from_3D(cam, edge[i], &edge2D[i].x, &edge2D[i].y);
		edge2D[i].z = 0;
		x2D[i] = edge2D[i].x;
		y2D[i] = edge2D[i].y;

	}

	//*_xmin = *std::min_element(x2D, x2D + 7);
	//*_ymin = *std::min_element(y2D, y2D + 7);

	//*_xmax = *std::max_element(x2D, x2D + 7);
	//*_ymax = *std::max_element(y2D, y2D + 7);

	//for (int i = 0; i < 8; ++i) {
	//	GRAPHICS::GET_SCREEN_COORD_FROM_WORLD_COORD(edge[i].x, edge[i].y, edge[i].z, &edge2D[i].x, &edge2D[i].y);
	//	edge2D[i].z = 0;
	//	x2D[i] = edge2D[i].x;
	//	y2D[i] = edge2D[i].y;

	//}

	float xmin, ymin, xmax, ymax;

	xmin = *std::min_element(x2D, x2D + 7);
	ymin = *std::min_element(y2D, y2D + 7);

	xmax = *std::max_element(x2D, x2D + 7);
	ymax = *std::max_element(y2D, y2D + 7);

	*_xmin = xmin * screenWidth;
	*_ymin = ymin * screenHeight;
	*_xmax = xmax * screenWidth;
	*_ymax = ymax * screenHeight;


}

void get_angles(Point cam, Vector3 world_coord, Vehicle vehicle, float* alpha, float* r_y) {

	// convert world coord to cam coord
	Vector3 coord_in_cam;
	world2cam(cam, world_coord, &coord_in_cam);

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Left
	Vector3 upVector, rightVector, forwardVector, position; //entity position
	Vector3 dim;

	float theta = atan(abs(coord_in_cam.x) / abs(coord_in_cam.z));

	get_vehicle_values(vehicle, &upVector, &rightVector, &forwardVector, &position, &dim);

	*r_y = atan(abs(forwardVector.z) / abs(forwardVector.x));

	*alpha = *r_y - theta;
}

void get_vehicle_values(Vehicle vehicle, Vector3* upVector, Vector3* rightVector, Vector3* forwardVector, Vector3* position, Vector3* dim) {

	ENTITY::GET_ENTITY_MATRIX(vehicle, rightVector, forwardVector, upVector, position);

	get_vehicle_dim(vehicle, dim);

}

void get_vehicle_dim(Vehicle vehicle, Vector3* dim) {

	Vector3 min, max;
	Hash model; //model hash

	model = ENTITY::GET_ENTITY_MODEL(vehicle);
	GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

	//calculate size
	(*dim).x = 0.5 * (max.x - min.x);
	(*dim).y = 0.5 * (max.y - min.y);
	(*dim).z = 0.5 * (max.z - min.z);
}

void world2cam(Point cam, Vector3 v, Vector3* d) {

	// translation
	float x = v.x - cam.cam_coord.x;
	float y = v.y - cam.cam_coord.y;
	float z = v.z - cam.cam_coord.z;

	// rotation
	float cam_x_rad = cam.cam_rot.x * (float)PI / 180.0f;
	float cam_y_rad = cam.cam_rot.y * (float)PI / 180.0f;
	float cam_z_rad = cam.cam_rot.z * (float)PI / 180.0f;

	// cos
	float cx = cos(cam_x_rad);
	float cy = cos(cam_y_rad);
	float cz = cos(cam_z_rad);

	// sin
	float sx = sin(cam_x_rad);
	float sy = sin(cam_y_rad);
	float sz = sin(cam_z_rad);

	// 转换为相机坐标系
	(*d).x = cy * (sz * y + cz * x) - sy * z;
	(*d).y = sx * (cy * z + sy * (sz * y + cz * x)) + cx * (cz * y - sz * x);
	(*d).z = cx * (cy * z + sy * (sz * y + cz * x)) - sx * (cz * y - sz * x);

}

void get_2D_from_3D_1(Vector3 world_coord, Point cam, float* x2D_pixel, float* y2D_pixel) {

	// convert to rad
	float cam_x_rad = cam.cam_rot.x * (float)PI / 180.0f;
	float cam_y_rad = cam.cam_rot.y * (float)PI / 180.0f;
	float cam_z_rad = cam.cam_rot.z * (float)PI / 180.0f;

	// sin
	float sx = sin(cam_x_rad);
	float sy = sin(cam_y_rad);
	float sz = sin(cam_z_rad);

	// cos
	float cx = cos(cam_x_rad);
	float cy = cos(cam_y_rad);
	float cz = cos(cam_z_rad);

	// R matrix
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;

	r11 = cz * cy;
	r12 = sz * cy;
	r13 = -sy;
	r21 = cz * sx * sy - sz * cx;
	r22 = sz * sx * sy + cz * cx;
	r23 = sx * cy;
	r31 = cz * cx * sy + sz * sx;
	r32 = sz * cx * sy - cz * sx;
	r33 = cx * cy;

	float x, y, z;	// temp, easy to write

	x = world_coord.x;
	y = world_coord.y;
	z = world_coord.z;

	//T matrix
	float t1, t2, t3;
	//t1 = world_coord.x - cam.cam_coord.x;
	//t2 = world_coord.y - cam.cam_coord.y;
	//t3 = world_coord.z - cam.cam_coord.z;

	t1 = world_coord.x - cam.cam_coord.x;
	t2 = world_coord.y - cam.cam_coord.y;
	t3 = world_coord.z - cam.cam_coord.z;


	Vector3 cam_coord;

	cam_coord.x = r11 * x + r12 * y + r13 * z + t1;
	cam_coord.y = r21 * x + r22 * y + r23 * z + t2;
	cam_coord.z = r31 * x + r32 * y + r33 * z + t3;


	// image coord
	float x2D, y2D;

	x = cam_coord.x;
	y = cam_coord.y;
	z = cam_coord.z;

	// get resolution
	int screenWidth = _screen_capture_worker.nScreenWidth;
	int screenHeight = _screen_capture_worker.nScreenHeight;

	// calculate focal length
	float CAM_FOV = CAM::GET_GAMEPLAY_CAM_FOV();
	float fov_rad = CAM_FOV * (float)PI / 180;
	float f = (screenHeight / 2.0f) * cos(fov_rad / 2.0f) / sin(fov_rad / 2.0f);

	x2D = f * x / z;
	y2D = f * y / z;

	// pixel coord

	float u0, v0;
	u0 = 1 / 2.0f;
	v0 = 1 / 2.0f;

	// pixel coord
	*x2D_pixel = x2D + u0 * screenWidth;
	*y2D_pixel = y2D + v0 * screenHeight;






}

void get_2D_from_3D(Point cam, Vector3 v, float* x2d, float* y2d)
{
	//3D到2D图像上的坐标转换

	Vector3 d;
	world2cam(cam, v, &d);

	// 获取屏幕分辨率
	int screenWidth = _screen_capture_worker.nScreenWidth;
	int screenHeight = _screen_capture_worker.nScreenHeight;

	float CAM_FOV = CAM::GET_GAMEPLAY_CAM_FOV();
	float fov_rad = CAM_FOV * (float)PI / 180;
	float f = (screenHeight / 2.0f) * cos(fov_rad / 2.0f) / sin(fov_rad / 2.0f);


	*x2d = ((d.x * (f / d.y)) / screenWidth + 0.5f);
	*y2d = (0.5f - (d.z * (f / d.y)) / screenHeight);
}

void resetExecuteTrajectory()
{
	_traj_idx = 0;
	_DO_FOLLOW_TRAJECTORY = false;
	_DO_CREATE_DENSE_TRAJECTORY = false;

	if (_ofile.is_open())
		_ofile.close();

	if (_DO_FOLLOW_DENSE_TRAJECTORY)
	{
		GDIReleaseScreenCapture();

		_DO_FOLLOW_DENSE_TRAJECTORY = false;
	}
}

void setParametersOfExecuteSparseTrajectory(bool create_dense_trajectory)
{
	_traj_idx = 0;
	_DO_FOLLOW_TRAJECTORY = true;
	if (create_dense_trajectory)
		_DO_CREATE_DENSE_TRAJECTORY = true;
	_DO_FOLLOW_DENSE_TRAJECTORY = false;
}

void setParametersOfExecuteDenseTrajectory()
{
	_traj_idx = 0;
	_DO_FOLLOW_TRAJECTORY = false;
	_DO_CREATE_DENSE_TRAJECTORY = false;
	_DO_FOLLOW_DENSE_TRAJECTORY = true;

	GDIInitScreenCapture(); // initialize screen capture object

}
float computeDistanceXY(Vector3 a, Vector3 b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void handleExecuteTrajectoryMenu(std::string menu_name)
{
	const int menu_item_number = 3;

	std::string menu_list[menu_item_number] = { "MOVE TO STARTING POINT", "EXECUTE SPARSE TRAJECTORY",
												"EXECUTE DENSE TRAJECTORY" };

	DWORD wait_time = 150;

	while (true)
	{
		// timed menu draw, used for pause after active line switch
		DWORD max_tick_count = GetTickCount() + wait_time;
		do
		{
			// draw menu
			drawMenuLine(menu_name, _MENU_LINE_WIDTH, 15.0, 18.0, 0.0, 5.0, false, true);
			for (int i = 0; i < menu_item_number; i++)
				if (i != _current_execute_trajectory_menu_index)
					drawMenuLine(menu_list[i], _MENU_LINE_WIDTH, 9.0, 60.0 + i * 36.0, 0.0, 9.0, false, false);
			drawMenuLine(menu_list[_current_execute_trajectory_menu_index],
				_MENU_LINE_WIDTH + 1.0, 11.0, 56.0 + _current_execute_trajectory_menu_index * 36.0, 0.0, 7.0, true, false);
			updateFeatures();
			WAIT(0);
		} while (GetTickCount() < max_tick_count);

		wait_time = 0;

		// listen if users press any buttons
		bool button_select, button_back, button_up, button_down;
		getButtonState(&button_select, &button_back, &button_up, &button_down);

		if (button_select) // if select button is pressed
		{
			switch (_current_execute_trajectory_menu_index)
			{
			case 0:
				moveToStartingPoint();
				break;
			case 1:
				if (readyExecuteSparseTrajectory())
				{
					setParametersOfExecuteSparseTrajectory(true);
				}
				break;
			case 2:
				if (readyExecuteDenseTrajectory())
				{
					setParametersOfExecuteDenseTrajectory();
				}
				break;
			}
			wait_time = 200;
		}
		else
			if (button_back || switchPressed()) // if back button is pressed
			{

				break;
			}
			else
				if (button_up) // if up/down button is pressed
				{

					if (_current_execute_trajectory_menu_index == 0)
						_current_execute_trajectory_menu_index = menu_item_number;
					_current_execute_trajectory_menu_index--;
					wait_time = 150;
				}
				else
					if (button_down)
					{

						_current_execute_trajectory_menu_index++;
						if (_current_execute_trajectory_menu_index == menu_item_number)
							_current_execute_trajectory_menu_index = 0;
						wait_time = 150;
					}

		if (_DO_FOLLOW_TRAJECTORY || _DO_FOLLOW_DENSE_TRAJECTORY)
			break;
	}
}


void handleMainMenu()
{
	const int menu_item_number = 2;
	std::string menu_name = "TRAJECTORY TOOL";
	std::string menu_list[menu_item_number] = { "CREATE TRAJECTORY", "EXECUTE TRAJECTORY" };

	DWORD wait_time = 150;

	while (true)
	{

		// timed menu draw, used for pause after active line switch
		DWORD max_tick_count = GetTickCount() + wait_time;
		do
		{
			// draw menu
			drawMenuLine(menu_name, _MENU_LINE_WIDTH, 15.0, 18.0, 0.0, 5.0, false, true);
			for (int i = 0; i < menu_item_number; i++)
				if (i != _current_main_menu_index)
					drawMenuLine(menu_list[i], _MENU_LINE_WIDTH, 9.0, 60.0 + i * 36.0, 0.0, 9.0, false, false);
			drawMenuLine(menu_list[_current_main_menu_index],
				_MENU_LINE_WIDTH + 1.0, 11.0, 56.0 + _current_main_menu_index * 36.0, 0.0, 7.0, true, false);
			updateFeatures();
			WAIT(0);
		} while (GetTickCount() < max_tick_count);

		wait_time = 0;

		// listen if users press any buttons
		bool button_select, button_back, button_up, button_down;
		getButtonState(&button_select, &button_back, &button_up, &button_down);

		if (button_select) // if users select one item within menu, check what the item is
		{
			switch (_current_main_menu_index)
			{
			case 0:
				handleCreateTrajectoryMenu(menu_list[0]);
				break;
			case 1:
				handleExecuteTrajectoryMenu(menu_list[1]);
				break;
			}
			wait_time = 200;
		}
		else
			if (button_back || switchPressed()) // if users press back
			{
				break;
			}
			else
				if (button_up) // if users press up/down
				{

					if (_current_main_menu_index == 0)
						_current_main_menu_index = menu_item_number;
					_current_main_menu_index--;
					wait_time = 150;
				}
				else
					if (button_down)
					{

						_current_main_menu_index++;
						if (_current_main_menu_index == menu_item_number)
							_current_main_menu_index = 0;
						wait_time = 150;
					}
		if (_DO_FOLLOW_TRAJECTORY || _DO_FOLLOW_DENSE_TRAJECTORY)
			break;
	}
}

void updateNotificationText()
{
	if (GetTickCount() < _notification_text_draw_ticks_max)
	{
		UI::SET_TEXT_FONT(0);
		UI::SET_TEXT_SCALE(0.55, 0.55);
		UI::SET_TEXT_COLOUR(255, 255, 255, 255);
		UI::SET_TEXT_WRAP(0.0, 1.0);
		UI::SET_TEXT_CENTRE(1);
		UI::SET_TEXT_DROPSHADOW(0, 0, 0, 0, 0);
		UI::SET_TEXT_EDGE(1, 0, 0, 0, 205);
		if (_notification_text_gxt_entry)
		{
			UI::_SET_TEXT_ENTRY((char*)_notification_text.c_str());
		}
		else
		{
			UI::_SET_TEXT_ENTRY("STRING");
			UI::_ADD_TEXT_COMPONENT_STRING((char*)_notification_text.c_str());
		}
		UI::_DRAW_TEXT(0.5, 0.5);
	}
}


int updateFeatures()
{
	updateNotificationText();
	return 0;
}

bool isFileExist(std::string file_text)
{
	std::ifstream ifile((file_text.c_str()));
	return ifile.is_open();
}

bool getCoordsFromMarker(Vector3& coords)
{
	Entity e = PLAYER::PLAYER_PED_ID();
	bool success = false;
	bool blipFound = false;

	// search for marker blip
	int blipIterator = UI::_GET_BLIP_INFO_ID_ITERATOR();
	for (Blip i = UI::GET_FIRST_BLIP_INFO_ID(blipIterator); UI::DOES_BLIP_EXIST(i) != 0; i = UI::GET_NEXT_BLIP_INFO_ID(blipIterator))
	{
		if (UI::GET_BLIP_INFO_ID_TYPE(i) == 4) // number 4 is the ID of marker on the built-in map
		{
			coords = UI::GET_BLIP_INFO_ID_COORD(i);
			success = true;;
			break;
		}
	}

	return success;
}


void main()
{

	createCamera();

	while (true)
	{

		if (switchPressed())
		{
			resetExecuteTrajectory();
			handleMainMenu();
		}

		if (_DO_FOLLOW_TRAJECTORY)
		{
			executeSparseTrajectory();
		}
		else
			if (_DO_FOLLOW_DENSE_TRAJECTORY)
			{
				executeDenseTrajectory();
			}

		updateFeatures();
		WAIT(0);

	}


}

void ScriptMain()
{
	srand(GetTickCount());
	main();
}
