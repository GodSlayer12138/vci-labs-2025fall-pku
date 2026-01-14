#define MIXBOX_LATENT_SIZE 7

typedef float mixbox_latent[MIXBOX_LATENT_SIZE];

void mixbox_lerp(unsigned char r1, unsigned char g1, unsigned char b1,
                 unsigned char r2, unsigned char g2, unsigned char b2,
                 float t,
                 unsigned char* out_r, unsigned char* out_g, unsigned char* out_b);
