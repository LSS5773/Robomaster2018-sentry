#include "beep.h"

/********************************************************************************
   ·äÃùÆ÷³ª¸è´óÀñ°ü
*********************************************************************************/
uint32_t Startup_Success_music_index = 0;
uint8_t warn = 0;
const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

//const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = 
//{
////  So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
//	Fa4M, Fa4M, Re2M, Re2M, La6M, La6M, La6M, Silent,
//};

const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = 
{
	Mi3M, Mi3M, Silent, Mi3M, Mi3M, Silent, Mi3M, Mi3M, Silent, So5M, So5M, So5M, So5M, Silent, Silent,
	Do1M, Do1M, Silent, Re2M, Re2M, Re2M, Re2M, Re2M, Silent, Silent, Fa4M, Fa4M, Silent, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent, Silent
};

void Sing(Sound_tone_e tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}
void Sing_Startup_music(uint32_t index)
{
  if(index < Startup_Success_music_len)
    Sing(Mavic_Startup_music[index]);
}
void warning(void) {
	if(warn) {
		Sing(Re2L);
		warn--;
		if(warn == 0) Sing(Silent);
	}
}
