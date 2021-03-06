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
#define IN_PICT_HISTOGRAM (1)
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn,
		struct OSC_VIS_REGIONS *regions, uint8 Color);
static unsigned long FrameNum=0;
void ProcessFrame(uint8 *pInputImg) {
	int c, r;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH / 2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);
	uint32 i1 = 0;
	uint32 Hist[256];
	uint8* p = data.u8TempImage[GRAYSCALE];
	uint32 w0 = 0;
	uint32 w1 = 0;
	uint32 M0 = 0;
	uint32 M1 = 0;
	float sigmaMax = 0;
	float sigma = 0;
	unsigned char dbg_err = 0;

	unsigned char Kmax = 0;
	unsigned char K= 0;
	memset(Hist, 0, sizeof(Hist));

	int Shift = 7;
	short Beta = 2;//the meaning is that in floating point the value of Beta is = 6/(1 << Shift) = 6/128 = 0.0469
	uint8 MaxForeground = 120;//the maximum foreground counter value (at 15 fps this corresponds to less than 10s)

	struct OSC_PICTURE Pic1, Pic2;//we require these structures to use Oscar functions
	struct OSC_VIS_REGIONS ImgRegions;//these contain the foreground objects

	if (data.ipc.state.nStepCounter == 1) {
		/* this is the first time we call this function */
		/* first time we call this; index 1 always has the background image */
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[GRAYSCALE],
				sizeof(data.u8TempImage[GRAYSCALE]));
		/* set foreground counter to zero */
		memset(data.u8TempImage[FGRCOUNTER], 0,
				sizeof(data.u8TempImage[FGRCOUNTER]));
	} else {

		/* this is the default case */
		for (i1 = 0; i1 < siz; i1++)/* we strongly rely on the fact that them images have the same size */
		{
			/* first determine the foreground estimate */
			//Hist[p[i1]] += 1;
			Hist[data.u8TempImage[GRAYSCALE][i1]] += 1;
		}
		FrameNum++;
		for (K = 0; K < 255; K++) {
			w0 = 0;
			w1 = 0;
			M0 = 0;
			M1 = 0;
			//OscLog(INFO, "K= %i Hist[K]= %i \n", K, Hist[K]);
			for (i1 = 0; i1 <= (K); i1++) {
				//OscLog(INFO, "K= %d w0= %d M0= %d i1= %d \n", K, w0, M0,i1);
				/*if(FrameNum == 50){
				OscLog(INFO, "K= %d w0= %d M0= %d i1= %d \n", K, w0, M0,i1);
				OscCall(Osc)
				}*/
				w0 += Hist[i1];
				M0 += Hist[i1] * i1;
			}
			if (w0 > M0) {
				dbg_err = 1;
			}
			for (i1 = K + 1; i1 <= 255; i1++) {
				w1 += Hist[i1];
				M1 += Hist[i1] * i1;
			}
			if (w0 > M0) {
				dbg_err = 2;
			}
			//division by zero
			if (w0 == 0 && w1 == 0) {
				sigma = (float) M0 - M1;
			} else if (w0 == 0) {
				sigma = (float) M0 - M1 / (float) w1;
			} else if (w1 == 0) {
				sigma = (float) M0 / (float) w0 - M1;
			} else {
				sigma = ((float) M0 / (float) w0) - (M1 / (float) w1);
				sigma = sigma *sigma* (w0 * (float)w1);
			}

			if (sigma > sigmaMax) {
				sigmaMax = sigma;
				Kmax = K;
			}
		}

		for (r = 0; r < siz; r += nc)/* we strongly rely on the fact that them images have the same size */
		{
			for (c = 0; c < nc; c++) {
				if(r/nc== Kmax/2 && c <nc/2){
					data.u8TempImage[GRAYSCALE][r + (2*c)] = 255;
					data.u8TempImage[GRAYSCALE][r + (2*c+1)] = 0;

				}

				switch (dbg_err) {
				case 0:
					/* first determine the foreground estimate */
					//				data.u8TempImage[THRESHOLD][r+c] = data.u8TempImage[GRAYSCALE][r+c] > data.ipc.state.nThreshold ? 0 : 0xff;
					data.u8TempImage[THRESHOLD][r + c]
							= data.u8TempImage[GRAYSCALE][r + c] > Kmax ? 0
									: 0xff;
					data.u8TempImage[GRAYSCALE][Kmax] = 0;
					data.u8TempImage[GRAYSCALE][Kmax+1] = 255;
					data.u8TempImage[GRAYSCALE][256] = 0;
					data.u8TempImage[GRAYSCALE][257] = 255;
					break;
				case 1:
					data.u8TempImage[GRAYSCALE][r+c] = 230;
					break;
				case 2:
					data.u8TempImage[GRAYSCALE][r+c] = 30;
					break;
				}

			}
#if IN_PICT_HISTOGRAM
			if (r < 128 * nc) {
				data.u8TempImage[GRAYSCALE][r + ((Hist[2 * r / nc] + Hist[2* r / nc + 1])>>5)] = 255;
				data.u8TempImage[GRAYSCALE][r + ((Hist[2 * r / nc] + Hist[2* r / nc + 1])>>5)+1] = 0;

			}
#endif
		}

		/*
		 {
		 //for debugging purposes we log the background values to console out
		 //we chose the center pixel of the image (adaption to other pixel is straight forward)
		 int offs = nc*(OSC_CAM_MAX_IMAGE_HEIGHT/2)/2+nc/2;

		 OscLog(INFO, "%d %d %d %d %d\n", (int) data.u8TempImage[GRAYSCALE][offs], (int) data.u8TempImage[BACKGROUND][offs], (int) data.u8TempImage[BACKGROUND][offs]-data.ipc.state.nThreshold,
		 (int) data.u8TempImage[BACKGROUND][offs]+data.ipc.state.nThreshold, (int) data.u8TempImage[FGRCOUNTER][offs]);
		 }
		 */


		// 1.st Dilatate
		for (r = nc; r < siz - nc; r += nc)/* we skip the first and last line */
		{
			for (c = 1; c < nc - 1; c++)/* we skip the first and last column */
			{
				unsigned char* p = &data.u8TempImage[THRESHOLD][r + c];
				data.u8TempImage[DILATION][r + c] = *(p - nc - 1) | *(p - nc)
						| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc
						- 1) | *(p + nc) | *(p + nc + 1);
			}
		}
		// 2nd Erode => Closing
		for (r = nc; r < siz - nc; r += nc)/* we skip the first and last line */
			{
				for (c = 1; c < nc - 1; c++)/* we skip the first and last column */
				{
					unsigned char* p = &data.u8TempImage[DILATION][r + c];
					data.u8TempImage[EROSION][r + c] = *(p - nc - 1) & *(p - nc)
							& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc
							- 1) & *(p + nc) & *(p + nc + 1);
				}
			}

		//wrap image EROSION in picture struct
		Pic1.data = data.u8TempImage[EROSION];
		Pic1.width = nc;
		Pic1.height = OSC_CAM_MAX_IMAGE_HEIGHT / 2;
		Pic1.type = OSC_PICTURE_GREYSCALE;
		//as well as DILATATION (will be used as output)
		Pic2.data = data.u8TempImage[DILATION];
		Pic2.width = nc;
		Pic2.height = OSC_CAM_MAX_IMAGE_HEIGHT / 2;
		Pic2.type = OSC_PICTURE_BINARY;//probably has no consequences
		//have to convert to OSC_PICTURE_BINARY which has values 0x01 (and not 0xff)
		OscVisGrey2BW(&Pic1, &Pic2, 0x80, false);

		//now do region labeling and feature extraction
		OscVisLabelBinary(&Pic2, &ImgRegions);
		OscVisGetRegionProperties(&ImgRegions);

		//OscLog(INFO, "number of objects %d\n", ImgRegions.noOfObjects);
		//plot bounding boxes both in gray and dilation image
		Pic2.data = data.u8TempImage[GRAYSCALE];
		OscVisDrawBoundingBoxBW(&Pic2, &ImgRegions, 255);
		OscVisDrawBoundingBoxBW(&Pic1, &ImgRegions, 128);
	}
}

/* Drawing Function for Bounding Boxes; own implementation because Oscar only allows colored boxes; here in Gray value "Color"  */
/* should only be used for debugging purposes because we should not drawn into a gray scale image */
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn,
		struct OSC_VIS_REGIONS *regions, uint8 Color) {
	uint16 i, o;
	uint8 *pImg = (uint8*) picIn->data;
	const uint16 width = picIn->width;
	for (o = 0; o < regions->noOfObjects; o++)//loop over regions
	{
		/* Draw the horizontal lines. */
		for (i = regions->objects[o].bboxLeft; i
				< regions->objects[o].bboxRight; i += 1) {
			pImg[width * regions->objects[o].bboxTop + i] = Color;
			pImg[width * (regions->objects[o].bboxBottom - 1) + i] = Color;
		}

		/* Draw the vertical lines. */
		for (i = regions->objects[o].bboxTop; i
				< regions->objects[o].bboxBottom - 1; i += 1) {
			pImg[width * i + regions->objects[o].bboxLeft] = Color;
			pImg[width * i + regions->objects[o].bboxRight] = Color;
		}
	}
	return SUCCESS;
}

