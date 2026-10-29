/* Host-side kinematics for the GEArm control UI (no hardware).
 *
 * The DH table is extracted from the firmware's robot.c at build time
 * (dh_from_firmware.h) so there is one source of truth. FK is the standard
 * DH chain; IK is damped least squares on the position Jacobian, seeded from
 * the caller's current joint angles. Orientation is left in the null space
 * because the UI's first goal is TCP position.
 *
 * stdio command protocol (length-prefixed replies, same shape as virtual_mc02):
 *   'F' u32 len + 6 doubles (joint degrees) -> x y z + 16 matrix doubles (mm)
 *   'I' u32 len + 3 or 6 doubles (x y z [roll pitch yaw] degrees) + 6 seed doubles
 *       -> 6 joint doubles (degrees) + residual + iterations
 *   'R' u32 len + u32 count -> round-trip self test: max FK->IK->FK error (mm)
 */
#include "dh_from_firmware.h"
#include <stddef.h>

__declspec(dllimport) int __cdecl _read(int, void *, unsigned int);
__declspec(dllimport) int __cdecl _write(int, const void *, unsigned int);
__declspec(dllimport) int __cdecl _setmode(int, int);
__declspec(dllimport) double __cdecl sin(double);
__declspec(dllimport) double __cdecl cos(double);
__declspec(dllimport) double __cdecl sqrt(double);
__declspec(dllimport) double __cdecl fabs(double);
__declspec(dllimport) void __cdecl exit(int);

#define O_BINARY 0x8000
#define NJ 6

static double dh_a[NJ], dh_alpha[NJ], dh_d[NJ], dh_off[NJ];

static void dh_init(void) {
  for (int i = 0; i < NJ; i++) {
    dh_a[i] = D_H[i][0];
    dh_alpha[i] = D_H[i][1];
    dh_d[i] = D_H[i][2];
    dh_off[i] = D_H[i][3];
  }
}

/* Standard DH: A = Rz(theta) Tz(d) Tx(a) Rx(alpha), theta = q + offset. */
static void fk(const double q[NJ], double T[4][4]) {
  double result[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  for (int i = 0; i < NJ; i++) {
    double th = q[i] + dh_off[i];
    double ct = cos(th), st = sin(th);
    double ca = cos(dh_alpha[i]), sa = sin(dh_alpha[i]);
    double A[4][4] = {
      {ct, -st * ca, st * sa, dh_a[i] * ct},
      {st, ct * ca, -ct * sa, dh_a[i] * st},
      {0, sa, ca, dh_d[i]},
      {0, 0, 0, 1}
    };
    double out[4][4];
    for (int r = 0; r < 4; r++)
      for (int c = 0; c < 4; c++) {
        double sum = 0;
        for (int k = 0; k < 4; k++) sum += result[r][k] * A[k][c];
        out[r][c] = sum;
      }
    for (int r = 0; r < 4; r++)
      for (int c = 0; c < 4; c++) result[r][c] = out[r][c];
  }
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++) T[r][c] = result[r][c];
}

static void tcp(const double q[NJ], double p[3]) {
  double T[4][4];
  fk(q, T);
  p[0] = T[0][3];
  p[1] = T[1][3];
  p[2] = T[2][3];
}

/* Numerical Jacobian of the TCP position with respect to the joints. */
static void jacobian(const double q[NJ], double J[3][NJ]) {
  const double h = 1e-6;
  double p0[3], p1[3];
  tcp(q, p0);
  for (int j = 0; j < NJ; j++) {
    double qq[NJ];
    for (int k = 0; k < NJ; k++) qq[k] = q[k];
    qq[j] += h;
    tcp(qq, p1);
    for (int r = 0; r < 3; r++) J[r][j] = (p1[r] - p0[r]) / h;
  }
}

/* Damped least squares: minimise ||J*dq - e|| + lambda*||dq||. */
static int dls(const double target[3], double q[NJ], int max_iter, double *residual) {
  double lambda = 1e-3;
  for (int iter = 0; iter < max_iter; iter++) {
    double p[3], e[3];
    tcp(q, p);
    for (int r = 0; r < 3; r++) e[r] = target[r] - p[r];
    *residual = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    if (*residual < 1e-4) return iter;
    double J[3][NJ];
    jacobian(q, J);
    /* JtJ + lambda*I, then solve 6x6 by Gauss elimination with Jt*e. */
    double A[NJ][NJ], b[NJ];
    for (int r = 0; r < NJ; r++) {
      b[r] = 0;
      for (int k = 0; k < 3; k++) b[r] += J[k][r] * e[k];
      for (int c = 0; c < NJ; c++) {
        double sum = 0;
        for (int k = 0; k < 3; k++) sum += J[k][r] * J[k][c];
        A[r][c] = sum + (r == c ? lambda : 0);
      }
    }
    for (int col = 0; col < NJ; col++) {
      int piv = col;
      for (int r = col + 1; r < NJ; r++)
        if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
      if (fabs(A[piv][col]) < 1e-12) return -1;
      if (piv != col) {
        for (int c = 0; c < NJ; c++) { double t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
        double t = b[col]; b[col] = b[piv]; b[piv] = t;
      }
      for (int r = col + 1; r < NJ; r++) {
        double f = A[r][col] / A[col][col];
        if (f == 0) continue;
        for (int c = col; c < NJ; c++) A[r][c] -= f * A[col][c];
        b[r] -= f * b[col];
      }
    }
    double dq[NJ];
    for (int r = NJ - 1; r >= 0; r--) {
      double sum = b[r];
      for (int c = r + 1; c < NJ; c++) sum -= A[r][c] * dq[c];
      dq[r] = sum / A[r][r];
    }
    /* Step limiting keeps the solver stable near singularities. */
    double step = 0;
    for (int r = 0; r < NJ; r++) step += dq[r] * dq[r];
    step = sqrt(step);
    double scale = step > 0.2 ? 0.2 / step : 1.0;
    for (int r = 0; r < NJ; r++) q[r] += dq[r] * scale;
  }
  return max_iter;
}

/* Try several seeds and keep the best result: a local solver on a redundant
 * arm has local minima, so multi-start is required for a usable IK. */
static int solve_best(const double target[3], const double seed[NJ], double q_out[NJ], double *best_res) {
  static const double seeds[5][NJ] = {
    {0, 0, 0, 0, 0, 0},
    {0.3, 0.3, 0.3, 0, 0, 0},
    {-0.3, -0.3, -0.3, 0, 0, 0},
    {0.6, -0.3, 0.4, 0, 0, 0},
    {-0.6, 0.3, -0.4, 0, 0, 0}
  };
  double best[NJ];
  double best_r = 1e30;
  int found = -1;
  for (int attempt = 0; attempt < 6; attempt++) {
    double q[NJ];
    if (attempt == 0) {
      for (int j = 0; j < NJ; j++) q[j] = seed[j];
    } else {
      for (int j = 0; j < NJ; j++) q[j] = seeds[attempt - 1][j];
    }
    double residual = 0;
    int iters = dls(target, q, 400, &residual);
    if (iters >= 0 && residual < best_r) {
      best_r = residual;
      for (int j = 0; j < NJ; j++) best[j] = q[j];
      found = iters;
      if (residual < 1e-4) break;
    }
  }
  for (int j = 0; j < NJ; j++) q_out[j] = best[j];
  *best_res = best_r;
  return found;
}

static int read_exact(void *dst, unsigned n) {
  unsigned char *d = dst;
  unsigned got = 0;
  while (got < n) {
    int r = _read(0, d + got, n - got);
    if (r <= 0) return 0;
    got += (unsigned)r;
  }
  return 1;
}
static void write_all(const void *src, unsigned n) {
  const unsigned char *s = src;
  unsigned put = 0;
  while (put < n) {
    int r = _write(1, s + put, n - put);
    if (r <= 0) exit(3);
    put += (unsigned)r;
  }
}
static void emit(const void *data, unsigned n) {
  unsigned char hdr[4];
  hdr[0] = (unsigned char)n; hdr[1] = (unsigned char)(n >> 8);
  hdr[2] = (unsigned char)(n >> 16); hdr[3] = (unsigned char)(n >> 24);
  write_all(hdr, 4);
  if (n) write_all(data, n);
}
static double get_d(const unsigned char *p) {
  /* little-endian IEEE-754 double, byte assembled to stay freestanding */
  double v;
  unsigned char *d = (unsigned char *)&v;
  for (int i = 0; i < 8; i++) d[i] = p[i];
  return v;
}
static void put_d(unsigned char *p, double v) {
  unsigned char *s = (unsigned char *)&v;
  for (int i = 0; i < 8; i++) p[i] = s[i];
}

/* Deterministic pseudo-random source so self-tests are reproducible. */
static unsigned long rng_state = 12345u;
static double rng(void) {
  rng_state = rng_state * 1103515245u + 12345u;
  return (double)((rng_state >> 16) & 0x7fffu) / 32767.0;
}

int main(void) {
  _setmode(0, O_BINARY);
  _setmode(1, O_BINARY);
  dh_init();
  static unsigned char in[512];
  for (;;) {
    unsigned char cmd;
    int r = _read(0, &cmd, 1);
    if (r <= 0) exit(0);
    unsigned char hdr[4];
    if (!read_exact(hdr, 4)) exit(0);
    unsigned n = (unsigned)hdr[0] | ((unsigned)hdr[1] << 8) | ((unsigned)hdr[2] << 16) | ((unsigned)hdr[3] << 24);
    if (n > sizeof in) exit(4);
    if (n && !read_exact(in, n)) exit(0);

    if (cmd == 'F' && n == 48u) {
      double q[NJ];
      for (int j = 0; j < NJ; j++) q[j] = get_d(in + 8 * j) * 3.14159265358979 / 180.0;
      /* Cumulative transforms give every joint origin for the UI skeleton. */
      double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
      static unsigned char out[8 * 21];
      int o = 0;
      for (int i = 0; i < NJ; i++) {
        double th = q[i] + dh_off[i];
        double ct = cos(th), st = sin(th);
        double ca = cos(dh_alpha[i]), sa = sin(dh_alpha[i]);
        double A[4][4] = {
          {ct, -st * ca, st * sa, dh_a[i] * ct},
          {st, ct * ca, -ct * sa, dh_a[i] * st},
          {0, sa, ca, dh_d[i]},
          {0, 0, 0, 1}
        };
        double R[4][4];
        for (int r = 0; r < 4; r++)
          for (int c = 0; c < 4; c++) {
            double sum = 0;
            for (int k = 0; k < 4; k++) sum += T[r][k] * A[k][c];
            R[r][c] = sum;
          }
        for (int r = 0; r < 4; r++)
          for (int c = 0; c < 4; c++) T[r][c] = R[r][c];
        put_d(out + 8 * (o++), T[0][3]);
        put_d(out + 8 * (o++), T[1][3]);
        put_d(out + 8 * (o++), T[2][3]);
      }
      put_d(out + 8 * (o++), T[0][3]);
      put_d(out + 8 * (o++), T[1][3]);
      put_d(out + 8 * (o++), T[2][3]);
      emit(out, sizeof out);
      continue;
    }
    if (cmd == 'I' && (n == 72u || n == 120u)) {
      int with_rot = (n == 120u);
      double target[3], seed[NJ];
      for (int i = 0; i < 3; i++) target[i] = get_d(in + 8 * i);
      const unsigned char *seedp = in + 8 * (with_rot ? 9 : 3);
      for (int j = 0; j < NJ; j++) seed[j] = get_d(seedp + 8 * j) * 3.14159265358979 / 180.0;
      double q[NJ], residual = 0;
      int iters = solve_best(target, seed, q, &residual);
      static unsigned char out[8 * 8];
      for (int j = 0; j < NJ; j++) put_d(out + 8 * j, q[j] * 180.0 / 3.14159265358979);
      put_d(out + 48, residual);
      put_d(out + 56, (double)(iters < 0 ? -1 : iters));
      emit(out, sizeof out);
      continue;
    }
    if (cmd == 'R') {
      /* Round trip: pick a reachable pose, run IK from a neutral seed, then FK. */
      double worst = 0;
      unsigned count = n >= 4 ? (unsigned)in[0] | ((unsigned)in[1] << 8) | ((unsigned)in[2] << 16) | ((unsigned)in[3] << 24) : 50u;
      if (count > 500u) count = 500u;
      if (!count) count = 50u;
      double worst_fail = 0;
      unsigned failures = 0;
      for (unsigned t = 0; t < count; t++) {
        double q[NJ], target[3];
        for (int j = 0; j < NJ; j++) q[j] = (rng() * 2.0 - 1.0) * 1.5;
        tcp(q, target);
        double seed[NJ];
        for (int j = 0; j < NJ; j++) seed[j] = 0;
        double residual = 0;
        (void)solve_best(target, seed, q, &residual);
        if (residual > 1e-3) failures++;
        double p[3];
        tcp(q, p);
        double err = 0;
        for (int i = 0; i < 3; i++) {
          double d = p[i] - target[i];
          err += d * d;
        }
        err = sqrt(err);
        if (err > worst) worst = err;
        if (residual > worst_fail) worst_fail = residual;
      }
      static unsigned char out[24];
      put_d(out, worst);
      put_d(out + 8, worst_fail);
      put_d(out + 16, (double)failures);
      emit(out, sizeof out);
      continue;
    }
    exit(5);
  }
}
