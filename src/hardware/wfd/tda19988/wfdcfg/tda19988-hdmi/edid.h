/*
 * See https://en.wikipedia.org/wiki/Extended_Display_Identification_Data#Detailed_Timing_Descriptor
 */

struct detailed_timing {
	/* clock will be 0 if it's a display desc instead of timing desc */
	uint16_t pixel_clock;

	uint8_t hactive_lsbs;
	uint8_t hblanking_lsbs;
	/* bits 7-4 hactive msbs, bits 3-0 hblanking msbs */
	uint8_t h_msbs;

	uint8_t vactive_lsbs;
	uint8_t vblanking_lsbs;
	/* bits 7-4 vactive msbs, bits 3-0 vblanking msbs */
	uint8_t vlines_msbs;

	uint8_t hfp_lsbs;
	uint8_t hsync_lsbs;

	/* bits 7-4 vfp lsbs, bits 3-0 vsync lsbs */
	uint8_t v_msbs;

	/* bits 7-6 hfp msbs, 5-4 hsync msbs, 3-2 vfp msbs, 1-0 vsync msbs */
	uint8_t additional_msbs;

	uint8_t hsize_lsbs;
	uint8_t vsize_lsbs;

	/* bits 7-4 hsize msbs, bits 3-0 vsize msbs */
	uint8_t size_msbs;

	uint8_t hborder;
	uint8_t vborder;

	uint8_t features
};

/* features bits */
#define FEATURE_INTERLACED		(1 << 7)
#define FEATURE_DIGITAL_SYNC_COMP_MASK	(3 << 3)
#define FEATURE_DIGITAL_SYNC_COMPOSITE	(2 << 3)
#define FEATURE_DIGITAL_SYNC_SEPARATE	(2 << 3)
#define FEATURE_VSYNC_POLARITY_MASK	(2 << 7) /* only valid for composite */
#define FEATURE_VSYNC_POSITIVE		(2 << 7)
#define FEATURE_HSYNC_POLARITY_MASK	(1 << 7) /* only valid for separate */
#define FEATURE_HSYNC_POSITIVE		(1 << 7)
