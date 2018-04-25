/* Font data for Consolas 16pt */
//typedef struct{
//	uint8_t width;
//	uint16_t offset;
//}FONT_INFO;
typedef struct{
	uint8_t width;
	uint16_t offset;
}FONT_CHAR_INFO;
//extern const FONT_INFO consolas_16ptFontInfo;
extern const uint8_t* consolas_16ptFontInfo;
extern const FONT_CHAR_INFO consolas_16ptDescriptors[];
