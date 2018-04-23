typedef struct {
  unsigned char duty;  
  unsigned char a_length;
  unsigned char d_length;
  unsigned char d_step;
  unsigned char s_length;
  unsigned char r_length;
} instr;

static const instr instruments[2] = {
  {50, 1, 2, 10, 2, 16},
  {10, 0, 0, 0, 0, 0},
};