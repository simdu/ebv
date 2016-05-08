/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

int TextColor = CYAN;

float bgrImg[IMG_SIZE];

const float avgFac = 0.95;

/* skip pixel at border */
const int Border = 2;

/* after this number of steps object is set to background */
const int frgLimit = 100;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

//Configure thresholds and bin texts. Allows easy change of thresholds.
double AngleThesholdDEG[] = { 0, 22.5, 67.5, 112.5, 157.5, 180 };
char *AngleText[] = { "0 deg", "45 deg", "90 deg", "135 deg" };

//Helpers for binning
double AngleTheshold[(sizeof(AngleThesholdDEG) / sizeof(double))];
uint16 AngleBins[sizeof(AngleTheshold) / sizeof(double) - 1];
uint8 NrOfBins = sizeof(AngleBins) / sizeof(uint16);

void ChangeDetection();
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void DetectAngles();
void BinAngle(double angle);
void ProcessAngle(uint16 CurObject);


void ResetProcess() {
}

void ProcessFrame() {
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		uint8 i;
		//calc rad values for the given deg values (only once)
		for (i = 0; i < (sizeof(AngleThesholdDEG) / sizeof(double)); i++) {
			AngleTheshold[i] = AngleThesholdDEG[i] * M_PI / 180;
		}

	} else {
		ChangeDetection();
		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);
		DetectRegions();
		DrawBoundingBoxes();
		DetectAngles();
	}
}

void ChangeDetection() {
	int r, c;
//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter in x-direction */
			int32 dx = -(int32) *(p - nc - 1) + (int32) *(p - nc + 1)
					- 2 * (int32) *(p - 1) + 2 * (int32) *(p + 1)
					- (int32) *(p + nc - 1) + (int32) *(p + nc + 1);
			int32 dy = (int32) *(p - nc - 1) + 2 * (int32) *(p - nc)
					+ (int32) *(p - nc + 1) - (int32) *(p + nc - 1)
					- 2 * (int32) *(p - nc) - (int32) *(p + nc + 1);
			/* check if norm is larger than threshold */
			int32 df2 = dx * dx + dy * dy;
			int32 thr2 = data.ipc.state.nThreshold * data.ipc.state.nThreshold;
			if (df2 > thr2) { //avoid square root
				//set pixel value to 255 in THRESHOLD image for gui
				data.u8TempImage[THRESHOLD][r + c] = 255;
			}
			//store derivatives (int16 is enough)
			imgDx[r + c] = (int16) dx;
			imgDy[r + c] = (int16) dy;

			//possibility to visualize data
			data.u8TempImage[BACKGROUND][r + c] = (uint8) MAX(0,
					MIN(255, 128+dx));
		}
	}
}

void Erode_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) & *(p - nc)
					& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc - 1)
					& *(p + nc) & *(p + nc + 1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) | *(p - nc)
					| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc - 1)
					| *(p + nc) | *(p + nc + 1);
		}
	}
}

void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
}

void DrawBoundingBoxes() {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, GREEN);
		}
	}
}

void DetectAngles() {
	//loop over objects
	uint16 o, c;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		//get pointer to root run of current object
		struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
		//loop over runs of current object
		do {
			//loop over pixel of current run
			for (c = currentRun->startColumn; c <= currentRun->endColumn; c++) {
				int r = currentRun->row;
				//processing for individual pixel at row r and column c
				double angle = atan2(imgDy[r * nc + c], imgDx[r * nc + c]);
				if (angle < 0) {   //determine the angle (atan2 gives back the whole interval ]-pi , pi[ )
					angle += M_PI;
				}
				BinAngle(angle);
			}
			currentRun = currentRun->next; //get net run of current object
		} while (currentRun != NULL); //end of current object
		ProcessAngle(o);
	}

}

void BinAngle(double angle) {
	uint8 i;
	//easier handling of multiple bins. For 4 bins it would may be faster using if/else or mod division
	for (i = 0; i < (NrOfBins); i++)
	{
		if ((angle >= AngleTheshold[i]) && (angle < AngleTheshold[i + 1])) {
			AngleBins[i]++;
			return; //nasty return, but optimized for speed
		}
	}
}

void ProcessAngle(uint16 CurObject) {
	uint8 i;
	uint8 MaxBinIdx = 0;
	//find max bin
	AngleBins[0] += AngleBins[NrOfBins - 1]; // 0° to 22.5 and 157.5° to 180° are the same bin
	AngleBins[NrOfBins - 1] = 0;
	for (i = 0; i < (NrOfBins - 1); i++) {
		if (AngleBins[i] > AngleBins[MaxBinIdx]) {
			MaxBinIdx = i;
		}
	}
	//label angle at center
	DrawString(ImgRegions.objects[CurObject].centroidX,
			ImgRegions.objects[CurObject].centroidY,
			strlen(AngleText[MaxBinIdx]), TINY, TextColor,
			AngleText[MaxBinIdx]);

	for (i = 0; i < (NrOfBins); i++) {
		AngleBins[i] = 0;
	}

}
