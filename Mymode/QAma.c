#include "QAma.h"                  // Device header

volatile Scan_information scan_information;
uint8_t QAtask_flag = 0;
int detected_colors[5];
int detected_championcup[3];

/** 颜色滤波窗口长度，必须为奇数。 */
#define COLOR_FILTER_WINDOW_SIZE 1U

/** 黑色最大明度，HSV 的 V 范围为 0 至 255。 */
#define COLOR_BLACK_VALUE_MAX 110U

/** 白色最小明度，HSV 的 V 范围为 0 至 255。 */
#define COLOR_WHITE_VALUE_MIN 220U

/** 白色最大饱和度，HSV 的 S 范围为 0 至 255。 */
#define COLOR_WHITE_SATURATION_MAX 35U

/** 彩色最小饱和度，低于该值时输出未知颜色。 */
#define COLOR_CHROMATIC_SATURATION_MIN 55U

/** 颜色编号：蓝、绿、红、白、黑。 */
#define COLOR_BLUE 1U
#define COLOR_GREEN 2U
#define COLOR_RED 3U
#define COLOR_WHITE 4U
#define COLOR_BLACK 5U

/** RGB 中值滤波和颜色投票状态。 */
typedef struct {
	uint8_t red[COLOR_FILTER_WINDOW_SIZE];
	uint8_t green[COLOR_FILTER_WINDOW_SIZE];
	uint8_t blue[COLOR_FILTER_WINDOW_SIZE];
	uint8_t classified[COLOR_FILTER_WINDOW_SIZE];
	uint8_t write_index;
	uint8_t sample_count;
	uint32_t last_sequence;
} ColorFilterState_t;

static ColorFilterState_t color_filter;
//// 任务 1 二维码到移动方案映射
//char* task1_scheme[16][5] = {
//    /* 1 */ {"Black", "White", "Red", "Green", "Blue"},
//    /* 2 */ {"White", "Black", "Red", "Green", "Blue"},
//    /* 3 */ {"White", "Black", "Green", "Red", "Blue"},
//    /* 4 */ {"Blue", "White", "Black", "Red", "Green"},
//    /* 5 */ {"White", "Red", "Blue", "Black", "Green"},
//    /* 6 */ {"Black", "Red", "Blue", "White", "Green"},
//    /* 7 */ {"Blue", "Green", "Black", "White", "Red"},
//    /* 8 */ {"Green", "White", "Blue", "Black", "Red"},
//    /* 9 */ {"White", "Green", "Black", "Blue", "Red"},
//    /* 10 */ {"Black", "Red", "Blue", "Green", "White"},
//    /* 11 */ {"Red", "Blue", "Green", "Black", "White"},
//    /* 12 */ {"Green", "Red", "Black", "Blue", "White"},
//    /* 13 */ {"White", "Red", "Blue", "Green", "Black"},
//    /* 14 */ {"Red", "Green", "White", "Blue", "Black"},
//    /* 15 */ {"Blue", "White", "Green", "Red", "Black"},
//    /* 16 */ {"Green", "Blue", "Red", "White", "Black"}
//};

//// 任务 2 二维码到移动方案映射
//char* task2_scheme[6][3] = {
//    /* 1 */ {"A", "B", "C"},
//    /* 2 */ {"A", "C", "B"},
//    /* 3 */ {"B", "A", "C"},
//    /* 4 */ {"B", "C", "A"},
//    /* 5 */ {"C", "A", "B"},
//    /* 6 */ {"C", "B", "A"}
//};
// 颜色编码规则（int型数组）：
// Black → 5 | White → 4 | Red → 3 | Green → 2 | Blue → 1
int task1_scheme[16][5] = {
    /* 0 */ {5, 4, 3, 2, 1},
    /* 1 */ {4, 5, 3, 2, 1},
    /* 2 */ {4, 5, 2, 3, 1},
    /* 3 */ {1, 4, 5, 3, 2},
    /* 4 */ {4, 3, 1, 5, 2},
    /* 5 */ {5, 3, 1, 4, 2},
    /* 6 */ {1, 2, 5, 4, 3},
    /* 7 */ {2, 4, 1, 5, 3},
    /* 8 */ {4, 2, 5, 1, 3},
    /* 9  */ {5, 3, 1, 2, 4},
    /* 10 */ {3, 1, 2, 5, 4},
    /* 11 */ {2, 3, 5, 1, 4},
    /* 12 */ {4, 3, 1, 2, 5},
    /* 13 */ {3, 2, 4, 1, 5},
    /* 14 */ {1, 4, 2, 3, 5},
    /* 15 */ {2, 1, 3, 4, 5}
};     

// 字母编码规则（int型数组）：
// A → 1 | B → 2 | C → 3
int task2_scheme[6][3] = {
    /* 1 */ {1, 2, 3},
    /* 2 */ {1, 3, 2},
//    /* 3 */ {2, 1, 3},
	/* 3 */ {2, 1, 3},
    /* 4 */ {2, 3, 1},
    /* 5 */ {3, 1, 2},
    /* 6 */ {3, 2, 1}
};

/**
 * @brief 计算一个 RGB 通道窗口的中值。
 */
static uint8_t color_median(const uint8_t *samples, uint8_t count)
{
	uint8_t sorted[COLOR_FILTER_WINDOW_SIZE];
	uint8_t i;

	for (i = 0U; i < count; i++) {
		sorted[i] = samples[i];
	}
	for (i = 1U; i < count; i++) {
		uint8_t value = sorted[i];
		uint8_t index = i;

		while ((index > 0U) && (sorted[index - 1U] > value)) {
			sorted[index] = sorted[index - 1U];
			index--;
		}
		sorted[index] = value;
	}

	return sorted[count / 2U];
}

/**
 * @brief 将 RGB 转换为整数 HSV 并识别黑、白、红、绿、蓝。
 *
 * H 的范围为 0 至 359 deg，S 和 V 的范围为 0 至 255。
 */
static uint8_t color_classify(uint8_t red, uint8_t green, uint8_t blue,
		uint16_t *hue_deg, uint8_t *saturation, uint8_t *value)
{
	uint8_t max_channel = red;
	uint8_t min_channel = red;
	uint8_t delta;
	uint8_t saturation_value;
	int32_t hue_value = 0;

	if (green > max_channel) {
		max_channel = green;
	}
	if (blue > max_channel) {
		max_channel = blue;
	}
	if (green < min_channel) {
		min_channel = green;
	}
	if (blue < min_channel) {
		min_channel = blue;
	}

	delta = max_channel - min_channel;
	saturation_value = (max_channel == 0U) ? 0U :
	        (uint8_t)((uint16_t)delta * 255U / max_channel);
	if (delta != 0U) {
		if (max_channel == red) {
			hue_value = 60 * ((int32_t)green - (int32_t)blue) / delta;
		} else if (max_channel == green) {
			hue_value = 120 +
			        60 * ((int32_t)blue - (int32_t)red) / delta;
		} else {
			hue_value = 240 +
			        60 * ((int32_t)red - (int32_t)green) / delta;
		}
		if (hue_value < 0) {
			hue_value += 360;
		}
	}

	*hue_deg = (uint16_t)hue_value;
	*saturation = saturation_value;
	*value = max_channel;

	if (max_channel <= COLOR_BLACK_VALUE_MAX) {
		return COLOR_BLACK;
	}
	if ((max_channel >= COLOR_WHITE_VALUE_MIN) &&
	        (saturation_value <= COLOR_WHITE_SATURATION_MAX)) {
		return COLOR_WHITE;
	}
	if (saturation_value < COLOR_CHROMATIC_SATURATION_MIN) {
		return 0U;
	}
	if ((hue_value < 60) || (hue_value >= 300)) {
		return COLOR_RED;
	}
	if (hue_value < 180) {
		return COLOR_GREEN;
	}
	return COLOR_BLUE;
}

/**
 * @brief 读取中断更新的一帧完整 RGB 数据。
 */
static uint8_t color_read_frame(uint8_t *red, uint8_t *green, uint8_t *blue)
{
	uint32_t sequence_before;
	uint32_t sequence_after;

	do {
		sequence_before = scan_information.sequence;
		if ((sequence_before & 1U) != 0U) {
			return 0U;
		}
		*red = scan_information.R;
		*green = scan_information.G;
		*blue = scan_information.B;
		sequence_after = scan_information.sequence;
	} while (sequence_before != sequence_after);

	if (sequence_after == color_filter.last_sequence) {
		return 0U;
	}
	color_filter.last_sequence = sequence_after;
	return 1U;
}

void Update_Materialcolor(void)
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t median_red;
	uint8_t median_green;
	uint8_t median_blue;
	uint8_t median_color;
	uint16_t hue_deg;
	uint8_t saturation;
	uint8_t value;
	uint8_t vote_count[COLOR_BLACK + 1U] = {0U};
	uint8_t winner = 0U;
	uint8_t winner_count = 0U;
	uint8_t required_votes;
	uint8_t i;

	if (!color_read_frame(&red, &green, &blue)) {
		return;
	}

	color_filter.red[color_filter.write_index] = red;
	color_filter.green[color_filter.write_index] = green;
	color_filter.blue[color_filter.write_index] = blue;
	color_filter.classified[color_filter.write_index] = color_classify(
	        red, green, blue, &hue_deg, &saturation, &value);
	color_filter.write_index = (uint8_t)((color_filter.write_index + 1U) %
	        COLOR_FILTER_WINDOW_SIZE);
	if (color_filter.sample_count < COLOR_FILTER_WINDOW_SIZE) {
		color_filter.sample_count++;
	}

	median_red = color_median(color_filter.red, color_filter.sample_count);
	median_green = color_median(color_filter.green, color_filter.sample_count);
	median_blue = color_median(color_filter.blue, color_filter.sample_count);
	median_color = color_classify(median_red, median_green, median_blue,
	        &hue_deg, &saturation, &value);
	scan_information.H = hue_deg;
	scan_information.S = saturation;
	scan_information.V = value;

	for (i = 0U; i < color_filter.sample_count; i++) {
		vote_count[color_filter.classified[i]]++;
	}
	for (i = 0U; i <= COLOR_BLACK; i++) {
		if (vote_count[i] > winner_count) {
			winner = i;
			winner_count = vote_count[i];
		}
	}
	required_votes = (uint8_t)(color_filter.sample_count / 2U + 1U);
	if ((winner_count >= required_votes) && (winner == median_color)) {
		scan_information.color = winner;
	}
}

void Get_Materialcolor(uint8_t ch)
{
	if (ch < 4U) {
		detected_colors[ch] = scan_information.color;
	} else if (ch == 4U) {
		detected_colors[ch] = 15 - detected_colors[1] - detected_colors[2] -
		        detected_colors[3] - detected_colors[0];
	}
}
