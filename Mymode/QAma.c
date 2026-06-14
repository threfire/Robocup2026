#include "QAma.h"                  // Device header

Scan_information scan_information;
uint8_t QAtask_flag = 0;
int detected_colors[5];
int detected_championcup[3];
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

//void Get_Materialcolor (uint8_t ch)
//{
//	if(ch<4){
//		if(scan_information.R<100&&scan_information.G<100&&scan_information.B<100){
//			detected_colors[ch] = 5;//黑
//			scan_information.color = detected_colors[ch];
//		}
//		else if(scan_information.R>200&&scan_information.G>200&&scan_information.B>200){
//			detected_colors[ch] = 4;//白
//			scan_information.color = detected_colors[ch];
//		}
//		else if(scan_information.R>130&&scan_information.G<100&&scan_information.B<100){
//			detected_colors[ch] = 3;//红
//			scan_information.color = detected_colors[ch];
//		}
//		else if(scan_information.R<100&&scan_information.G>130&&scan_information.B<130){
//			detected_colors[ch] = 2;//绿
//			scan_information.color = detected_colors[ch];
//		}
//		else if(scan_information.R<100&&scan_information.G<150&&scan_information.B>150){
//			detected_colors[ch] = 1;//蓝
//			scan_information.color = detected_colors[ch];
//		}
//		else
//		{scan_information.color = 0;}
//	}
//	else {
//		detected_colors[ch] = 15 - detected_colors[1]- detected_colors[2]- detected_colors[3]- detected_colors[0];}
//		
//}

void Get_Materialcolor (uint8_t ch)
{
	if(ch<4){
		if(scan_information.R<100&&scan_information.G<100&&scan_information.B<100){
			detected_colors[ch] = 5;//黑
			scan_information.color = detected_colors[ch];
		}
		else if(scan_information.R>180&&scan_information.G>180&&scan_information.B>180){
			detected_colors[ch] = 4;//白
			scan_information.color = detected_colors[ch];
		}
		else if(scan_information.R>120&&scan_information.G<100&&scan_information.B<100){
			detected_colors[ch] = 3;//红
			scan_information.color = detected_colors[ch];
		}
		else if(scan_information.R<110&&scan_information.G>120&&scan_information.B<140){
			detected_colors[ch] = 2;//绿
			scan_information.color = detected_colors[ch];
		}
		else if(scan_information.R<100&&scan_information.G<170&&scan_information.B>140){
			detected_colors[ch] = 1;//蓝
			scan_information.color = detected_colors[ch];
		}

	}
	else {
		detected_colors[ch] = 15 - detected_colors[1]- detected_colors[2]- detected_colors[3]- detected_colors[0];}
		
}
