/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "CHOP_CPlusPlusBase.h"

/*

This example file implements a class that does 2 different things depending on
if a CHOP is connected to the CPlusPlus CHOPs input or not.
The example is timesliced, which is the more complex way of working.

If an input is connected the node will output the same number of channels as the
input and divide the first 'N' samples in the input channel by 2. 'N' being the current
timeslice size. This is noteworthy because if the input isn't changing then the output
will look wierd since depending on the timeslice size some number of the first samples
of the input will get used.

If no input is connected then the node will output a smooth sine wave at 120hz.
*/

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <dlib/opencv.h>
//#include "dlib/opencv.h"
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
//#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>


// To get more help about these functions, look at CHOP_CPlusPlusBase.h
class FaceCHOP : public CHOP_CPlusPlusBase
{
public:
	FaceCHOP(const OP_NodeInfo* info);
	virtual ~FaceCHOP();

	virtual void		getGeneralInfo(CHOP_GeneralInfo*, const OP_Inputs*, void* ) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*, const OP_Inputs*, void*) override;
	virtual void		getChannelName(int32_t index, OP_String *name, const OP_Inputs*, void* reserved) override;

	virtual void		execute(CHOP_Output*,
								const OP_Inputs*,
								void* reserved) override;


	virtual int32_t		getNumInfoCHOPChans(void* reserved1) override;
	virtual void		getInfoCHOPChan(int index,
										OP_InfoCHOPChan* chan,
										void* reserved1) override;

	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize, void* resereved1) override;
	virtual void		getInfoDATEntries(int32_t index,
											int32_t nEntries,
											OP_InfoDATEntries* entries,
											void* reserved1) override;

	virtual void		setupParameters(OP_ParameterManager* manager, void *reserved1) override;
	virtual void		pulsePressed(const char* name, void* reserved1) override;

private:

	// We don't need to store this pointer, but we do for the example.
	// The OP_NodeInfo class store information about the node that's using
	// this instance of the class (like its name).
	const OP_NodeInfo*	myNodeInfo;

	// In this example this value will be incremented each time the execute()
	// function is called, then passes back to the CHOP 
	int32_t				myExecuteCount;


	// FaceCHOP functions
	cv::Point2d FaceCHOP::pToUV(dlib::full_object_detection shape, int index, double width, double height, double aspect);
	bool FaceCHOP::loadFaceLandmarks(const char* FaceCascadePar);
	void setup();
	virtual void getErrorString(OP_String* error, void* reserved1) override;
	virtual void getWarningString(OP_String* warning, void* reserved1) override;
	virtual void getInfoPopupString(OP_String* warning, void* reserved1) override;

	bool checkLandmarksFile(const char* Facelandmarksfile);

	int	myErrors = 0;

	OP_TOPInputDownloadOptions imageDownloadOptions;

	std::string	emptyString = "";

	bool hasSetup = false;
	bool hasLoadedLandmarks = false;
	dlib::frontal_face_detector detector; // this finds rectangles of faces
	dlib::shape_predictor predictor; // this finds the 68 landmarks on the faces given the rectangles

	// fill in 3D ref points (world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	std::vector<cv::Point3d> object_pts;

	// 2D ref points (image coordinates), referenced from detected facial feature
	std::vector<cv::Point2d> image_pts;

	// result
	cv::Mat rotation_vec = cv::Mat(3, 1, CV_32F, 0.);
	cv::Mat rotation_mat;
	cv::Mat translation_vec = cv::Mat(3, 1, CV_32F, 0.);
	cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);
	cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

	// temporary buffer for decomposeProjectionMatrix()
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

	bool solvePnP_success = false;

	int frameCounter = 0;
	std::vector<dlib::rectangle> faces;

	int MAXFACES = 8;
	int numFacesFound = 0;
};
