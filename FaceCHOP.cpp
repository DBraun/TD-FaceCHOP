/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

 /***************************************************************************
  *   Copyright (C) 2007 by pedromartins   *
  *   pedromartins@isr.uc.pt   *
  *                                                                         *
  *   This program is free software; you can redistribute it and/or modify  *
  *   it under the terms of the GNU General Public License as published by  *
  *   the Free Software Foundation; either version 2 of the License, or     *
  *   (at your option) any later version.                                   *
  *                                                                         *
  *   This program is distributed in the hope that it will be useful,       *
  *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
  *   GNU General Public License for more details.                          *
  *                                                                         *
  *   You should have received a copy of the GNU General Public License     *
  *   along with this program; if not, write to the                         *
  *   Free Software Foundation, Inc.,                                       *
  *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
  ***************************************************************************/

#include "FaceCHOP.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>

// OpenCV
using namespace cv;

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillCHOPPluginInfo(CHOP_PluginInfo *info)
{
	// Always set this to CHOPCPlusPlusAPIVersion.
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	// The opType is the unique name for this CHOP. It must start with a 
	// capital A-Z character, and all the following characters must lower case
	// or numbers (a-z, 0-9)
	info->customOPInfo.opType->setString("Facechop");

	// The opLabel is the text that will show up in the OP Create Dialog
	info->customOPInfo.opLabel->setString("Face CHOP");
	info->customOPInfo.opIcon->setString("FC");

	// Information about the author of this OP
	info->customOPInfo.authorName->setString("David Braun");
	info->customOPInfo.authorEmail->setString("github.com/DBraun");

	// This CHOP can work with 0 inputs
	info->customOPInfo.minInputs = 0;

	// It can accept up to 1 input though, which changes its behavior
	info->customOPInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new FaceCHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (FaceCHOP*)instance;
}

};


FaceCHOP::FaceCHOP(const OP_NodeInfo* info) : myNodeInfo(info)
{
	myExecuteCount = 0;
}

FaceCHOP::~FaceCHOP()
{

}

void
FaceCHOP::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved1)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = false;

	// Note: To disable timeslicing you'll need to turn this off, as well as ensure that
	// getOutputInfo() returns true, and likely also set the info->numSamples to how many
	// samples you want to generate for this CHOP. Otherwise it'll take on length of the
	// input CHOP, which may be timesliced.
	ginfo->timeslice = false;

	ginfo->inputMatchIndex = 0;
}

bool
FaceCHOP::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved1)
{
	info->numChannels = 4;
	info->numSamples = MAXFACES*71;
	// info->sampleRate = 60;
	return true;
}

void
FaceCHOP::getChannelName(int32_t index, OP_String *name, const OP_Inputs* inputs, void* reserved1)
{
	switch (index) {
		case 0:
			name->setString("x");
			break;
		case 1:
			name->setString("y");
			break;
		case 2:
			name->setString("z");
			break;
		case 3:
			name->setString("w");
			break;
	}
}

// return true if successfully loaded landmarks file. otherwise false
bool FaceCHOP::loadFaceLandmarks(const char* FaceCascadePar) {
	try
	{
		dlib::deserialize(FaceCascadePar) >> predictor;
	}
	catch (const std::exception&) {
		return false;
	}
	hasLoadedLandmarks = true;
	return true;
}

void FaceCHOP::setup() {

	// the default face detector. it's a HOG method, not CNN
	// so it's not great when faces are rotated.
	detector = dlib::get_frontal_face_detector();

	// convert centimeters to meters.
	double cmToMeters = 1. / 100.;

	// The numbers immediately below are copyright of Pedro Martins
	// Copyright(C) 2007 by pedromartins
	// http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	// http://aifi.isr.uc.pt/HeadPoseEstimation.html
	object_pts.push_back(cmToMeters * cv::Point3d(6.825897, 6.760612, 4.402142));    // #33 left brow left corner
	object_pts.push_back(cmToMeters * cv::Point3d(1.330353, 7.122144, 6.903745));    // #29 left brow right corner
	object_pts.push_back(cmToMeters * cv::Point3d(-1.330353, 7.122144, 6.903745));   // #34 right brow left corner
	object_pts.push_back(cmToMeters * cv::Point3d(-6.825897, 6.760612, 4.402142));   // #38 right brow right corner
	object_pts.push_back(cmToMeters * cv::Point3d(5.311432, 5.485328, 3.987654));    // #13 left eye left corner
	object_pts.push_back(cmToMeters * cv::Point3d(1.789930, 5.393625, 4.413414));    // #17 left eye right corner
	object_pts.push_back(cmToMeters * cv::Point3d(-1.789930, 5.393625, 4.413414));   // #25 right eye left corner
	object_pts.push_back(cmToMeters * cv::Point3d(-5.311432, 5.485328, 3.987654));   // #21 right eye right corner
	object_pts.push_back(cmToMeters * cv::Point3d(2.005628, 1.409845, 6.165652));    // #55 nose left corner
	object_pts.push_back(cmToMeters * cv::Point3d(-2.005628, 1.409845, 6.165652));   // #49 nose right corner
	object_pts.push_back(cmToMeters * cv::Point3d(2.774015, -2.080775, 5.048531));   // #43 mouth left corner
	object_pts.push_back(cmToMeters * cv::Point3d(-2.774015, -2.080775, 5.048531));  // #39 mouth right corner
	object_pts.push_back(cmToMeters * cv::Point3d(0.000000, -3.116408, 6.097667));   // #45 mouth central bottom corner
	object_pts.push_back(cmToMeters * cv::Point3d(0.000000, -7.415691, 4.070434));   // #6 chin corner

	// other one-time stuff
	imageDownloadOptions.cpuMemPixelType = OP_CPUMemPixelType::BGRA8Fixed;
	imageDownloadOptions.verticalFlip = true;  // openCV is upside-down compared to TouchDesigner so flip the image.

	hasSetup = true;
}

cv::Point2d FaceCHOP::pToUV(dlib::full_object_detection shape, int index, double width, double height, double aspect) {
	dlib::point p = shape.part(index);

	return cv::Point2d(p.x() / width - .5, (p.y() / height - .5) / aspect);
}

bool FaceCHOP::checkLandmarksFile(const char* Facelandmarksfile) {
	
	if (hasLoadedLandmarks) {
		return true;
	}

	if (emptyString.compare(Facelandmarksfile) == 0) {

		myErrors = 1;
		return false;
	}
	else if (!hasLoadedLandmarks) {
		if (!loadFaceLandmarks(Facelandmarksfile)) {
			myErrors = 2;
			return false;
		}
	}

	return true;
}

void
FaceCHOP::execute(CHOP_Output* output,
							  const OP_Inputs* inputs,
							  void* reserved)
{
	myExecuteCount++;
	myErrors = 0; // Reset errors
	numFacesFound = 0; // reset num found faces

	if (!hasSetup) {
		setup();
	}

	const char* Facelandmarksfile = inputs->getParFilePath("Facelandmarksfile");

	if (!checkLandmarksFile(Facelandmarksfile)) {
		return;
	}

	// Get the image that might have faces in it.
	const OP_TOPInput* input = inputs->getParTOP("Image");

	const char* Imagedownload = inputs->getParString("Imagedownload");
	if (!strcmp(Imagedownload, "Delayed")) {
		imageDownloadOptions.downloadType = OP_TOPInputDownloadType::Delayed;
	}
	else {
		imageDownloadOptions.downloadType = OP_TOPInputDownloadType::Instant;
	}

	const uchar* videoSrc = (const uchar*)inputs->getTOPDataInCPUMemory(input, &imageDownloadOptions);

	if (!videoSrc) {
		myErrors = 3;
		return;
	}

	double height = input->height;
	double width = input->width;

	// todo: avoid using openCV mat entirely.
	// Loading input buffer into cv::Mat
	Mat frame(height, width, CV_8UC4, (void*)videoSrc);
	Mat frameBGR;
	cvtColor(frame, frameBGR, COLOR_BGRA2BGR);

	solvePnP_success = false;

	dlib::cv_image<dlib::bgr_pixel> cimg(frameBGR);

	if (frameCounter == 0) {
		// Get rectangle bounding boxes of faces.
		// We could do this with openCV instead, but the landmark detection
		// was trained on a model with dlib's face rectangles, not openCV's.
		// More info: https://github.com/davisking/dlib-models
		double adjust_threshold = .0;
		faces = detector(cimg, adjust_threshold);
	}
	int modulus = inputs->getParInt("Facerectframeskip") + 1;
	frameCounter = (frameCounter + 1) % modulus;

	double aspect = width / height;
	const double math_pi = std::acos(-1);

	// Camera internals
	double cam_width = 1.;
	double fov = inputs->getParDouble("Fov");
	double fov_radians = fov * (math_pi / 180.);
	double focal_length = (cam_width / 2.) / std::tan(fov_radians / 2.);

	Point2d center = cv::Point2d(0., 0.);
	cv::Mat myCameraMatrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);

	cv::Mat_<double> distCoeffs(cv::Size(1, 4));
	distCoeffs = 0.f;

	numFacesFound = int(faces.size());

	bool Facelandmarkstoggle = bool(inputs->getParDouble("Facelandmarkstoggle"));

	for (int face_i = 0; face_i < MAXFACES; face_i++) {

		if (face_i < numFacesFound) {

			dlib::rectangle faceRect = faces[face_i];

			// first save the face rectangle
			int offset = 71 * face_i + 68;

			output->channels[0][offset] = .5 * (faceRect.right() + faceRect.left()) / width; // avg x normalized to pct
			output->channels[1][offset] = 1.-.5 * (faceRect.top() + faceRect.bottom()) / height; // avg y normalized to pct
			output->channels[2][offset] = (faceRect.right() - faceRect.left()) / width; // face size x normalize to pct
			output->channels[3][offset] = (faceRect.bottom() - faceRect.top()) / height; // face size y normalized to pct

			if (!Facelandmarkstoggle) {
				for (int i = 0; i < 68; i++) {
					offset = 71 * face_i + i;
					output->channels[0][offset] = 0.0f;
					output->channels[1][offset] = 0.0f;
					output->channels[2][offset] = 0.0f;
					output->channels[3][offset] = 0.0f;
				}
				continue;
			}
			
			dlib::full_object_detection shape = predictor(cimg, faceRect);

			for (int i = 0; i < 68; i++) {
				dlib::point p = shape.part(i);
				offset = 71 * face_i + i;
				output->channels[0][offset] =  ((p.x() / width) - .5);
				output->channels[1][offset] = -((p.y() / height) - .5) / aspect;
				output->channels[2][offset] = 0.0f;
				output->channels[3][offset] = 1.f;

			}

			// fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
			image_pts.push_back(pToUV(shape, 17, width, height, aspect)); //#17 left brow left corner
			image_pts.push_back(pToUV(shape, 21, width, height, aspect)); //#21 left brow right corner
			image_pts.push_back(pToUV(shape, 22, width, height, aspect)); //#22 right brow left corner
			image_pts.push_back(pToUV(shape, 26, width, height, aspect)); //#26 right brow right corner
			image_pts.push_back(pToUV(shape, 36, width, height, aspect)); //#36 left eye left corner
			image_pts.push_back(pToUV(shape, 39, width, height, aspect)); //#39 left eye right corner
			image_pts.push_back(pToUV(shape, 42, width, height, aspect)); //#42 right eye left corner
			image_pts.push_back(pToUV(shape, 45, width, height, aspect)); //#45 right eye right corner
			image_pts.push_back(pToUV(shape, 31, width, height, aspect)); //#31 nose left corner
			image_pts.push_back(pToUV(shape, 35, width, height, aspect)); //#35 nose right corner
			image_pts.push_back(pToUV(shape, 48, width, height, aspect)); //#48 mouth left corner
			image_pts.push_back(pToUV(shape, 54, width, height, aspect)); //#54 mouth right corner
			image_pts.push_back(pToUV(shape, 57, width, height, aspect)); //#57 mouth central bottom corner
			image_pts.push_back(pToUV(shape, 8, width, height, aspect));   //#8 chin corner

			// start with no rotation.
			rotation_vec = cv::Mat(3, 1, CV_32F, 0.);
			// important to start with a negative tz value.
			translation_vec = (cv::Mat_<float>(3, 1) << 0.f, 0.f, -1.f);

			solvePnP_success = cv::solvePnP(object_pts, image_pts, myCameraMatrix, distCoeffs, rotation_vec, translation_vec, 1);
			//solvePnP_success = solvePnPRansac(object_pts, image_pts, myCameraMatrix, distCoeffs, rotation_vec, translation_vec);
			image_pts.clear();

			if (solvePnP_success) {

				// calculate the euler angle
				cv::Rodrigues(rotation_vec, rotation_mat);
				cv::hconcat(rotation_mat, translation_vec, pose_mat);
				cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

				offset = 71 * face_i + 68;
				offset += 1;
				output->channels[0][offset] = -translation_vec.at<float>(0, 0);
				output->channels[1][offset] = translation_vec.at<float>(1, 0);
				output->channels[2][offset] = translation_vec.at<float>(2, 0);
				output->channels[3][offset] = 1.f;
				offset += 1;
				output->channels[0][offset] = euler_angle.at<double>(0);
				output->channels[1][offset] = -euler_angle.at<double>(1);
				output->channels[2][offset] = -euler_angle.at<double>(2);
				output->channels[3][offset] = 1.f;
			}
		}
		else {
			for (int i = 0; i < 71; i++) {
				int offset = face_i * 71 + i;
				output->channels[0][offset] = 0.f;
				output->channels[1][offset] = 0.f;
				output->channels[2][offset] = 0.f;
				output->channels[3][offset] = 0.f;
			}
		}
	}
}

// You can use this function to put the node into a error state
// by calling setSting() on 'error' with a non empty string.
// Leave 'error' unchanged to not go into error state.
void FaceCHOP::getErrorString(OP_String* error, void* reserved1) {

	switch (myErrors) {
		case 0:
			// no errors. must set string to blank.
			error->setString("");
			break;
		case 1:
			error->setString("shape_predictor_68_face_landmarks.dat File is missing");
			break;
		case 2:
			error->setString("Error while loading face landmarks file.");
			break;
		case 3:
			error->setString("No video input.");
			break;
	}

}

// You can use this function to put the node into a warning state
// by calling setSting() on 'warning' with a non empty string.
// Leave 'warning' unchanged to not go into warning state.
void FaceCHOP::getWarningString(OP_String* warning, void* reserved1)
{
}

// Use this function to return some text that will show up in the
// info popup (when you middle click on a node)
// call setString() on info and give it some info if desired.
void FaceCHOP::getInfoPopupString(OP_String* info, void* reserved1)
{
}

int32_t
FaceCHOP::getNumInfoCHOPChans(void * reserved1)
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send one channel.
	return 2;
}

void
FaceCHOP::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan,
										void* reserved1)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name->setString("executeCount");
		chan->value = (float)myExecuteCount;
	}

	if (index == 1) {
		chan->name->setString("numFacesFound");
		chan->value = numFacesFound;
	}
}

bool		
FaceCHOP::getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved1)
{
	infoSize->rows = 1;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
FaceCHOP::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries, 
										void* reserved1)
{
	char tempBuffer[4096];

	if (index == 0)
	{
		// Set the value for the first column
		entries->values[0]->setString("executeCount");

		// Set the value for the second column
#ifdef _WIN32
		sprintf_s(tempBuffer, "%d", myExecuteCount);
#else // macOS
        snprintf(tempBuffer, sizeof(tempBuffer), "%d", myExecuteCount);
#endif
		entries->values[1]->setString(tempBuffer);
	}

}

void
FaceCHOP::setupParameters(OP_ParameterManager* manager, void *reserved1)
{

	// Face Landmarks File
	{
		OP_StringParameter sp;

		sp.name = "Facelandmarksfile";
		sp.label = "Face Landmarks File";
		sp.defaultValue = "shape_predictor_68_face_landmarks.dat";
		OP_ParAppendResult res = manager->appendFile(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	// Field of View of the camera
	{
		OP_NumericParameter np;
		np.name = "Fov";
		np.label = "Field of View (Horizontal)";
		np.defaultValues[0] = 60.;

		np.minSliders[0] = 30;
		np.maxSliders[0] = 90;

		np.minValues[0] = 0.001;
		np.maxValues[0] = 180;

		np.clampMins[0] = true;
		np.clampMaxes[0] = true;

		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// input TOP
	{
		OP_StringParameter	sp;

		sp.name = "Image";
		sp.label = "Image";

		sp.defaultValue = "";

		OP_ParAppendResult res = manager->appendTOP(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	// toggle face landmarks
	{
		OP_NumericParameter	np;

		np.name = "Facelandmarkstoggle";
		np.label = "Face Landmarks";
		np.defaultValues[0] = 1.0;

		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Frame skip for face rectangle
	// value of 0 means update face rectangles EVERY frame
	// value of 1 means update face rectangles every other frame
	// value of 2 means update face rectangles once out of three frames
	{
		OP_NumericParameter	np;

		np.name = "Facerectframeskip";
		np.label = "Face Rectangle Frame Skip";
		np.defaultValues[0] = 1.0;
		np.clampMins[0] = true;
		np.minValues[0] = 0;
		np.minSliders[0] = 0;
		np.maxSliders[0] = 3;

		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Image download type
	{
		OP_StringParameter	sp;

		sp.name = "Imagedownload";
		sp.label = "Image Download";

		sp.defaultValue = "Instant";

		const char *names[] = { "Instant", "Delayed"};
		const char *labels[] = { "Instant", "Delayed"};

		OP_ParAppendResult res = manager->appendMenu(sp, 2, names, labels);
		assert(res == OP_ParAppendResult::Success);
	}

}

void 
FaceCHOP::pulsePressed(const char* name, void* reserved1)
{
	//if (!strcmp(name, "Reset"))
	//{
	//}
}
