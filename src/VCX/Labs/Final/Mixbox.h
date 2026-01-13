#define MIXBOX_LATENT_SIZE 7

typedef float mixbox_latent[MIXBOX_LATENT_SIZE];

void mixbox_lerp(unsigned char r1, unsigned char g1, unsigned char b1,
                 unsigned char r2, unsigned char g2, unsigned char b2,
                 float t,
                 unsigned char* out_r, unsigned char* out_g, unsigned char* out_b);

void mixbox_lerp_float(float r1, float g1, float b1,
                       float r2, float g2, float b2,
                       float t,
                       float* out_r, float* out_g, float* out_b);

void mixbox_lerp_linear_float(float r1, float g1, float b1,
                              float r2, float g2, float b2,
                              float t,
                              float* out_r, float* out_g, float* out_b);

void mixbox_rgb_to_latent(unsigned char r, unsigned char g, unsigned char b, mixbox_latent out_latent);
void mixbox_latent_to_rgb(mixbox_latent latent, unsigned char* out_r, unsigned char* out_g, unsigned char* out_b);

void mixbox_float_rgb_to_latent(float r, float g, float b, mixbox_latent out_latent);
void mixbox_latent_to_float_rgb(mixbox_latent latent, float* out_r, float* out_g, float* out_b);

void mixbox_linear_float_rgb_to_latent(float r, float g, float b, mixbox_latent out_latent);
void mixbox_latent_to_linear_float_rgb(mixbox_latent latent, float* out_r, float* out_g, float* out_b);
