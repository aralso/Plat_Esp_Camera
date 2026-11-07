
    .text

    .align  4
    .global residual_transform_4x4
    .type   residual_transform_4x4, @function

residual_transform_4x4:
    entry   a1, 64 # 128bits * 2

	# Move data to Q-registers
    ee.vld.128.ip       q0, a2, 0
    ee.vld.128.ip       q1, a3, 0

	# Horizontal pass
    ee.fft.r2bf.s16     q2, q3, q0, q1, 0
    ee.fft.r2bf.s16     q0, q1, q2, q3, 1
    ee.vzip.16          q0, q1
    ee.vzip.16          q0, q1
	# q2/q3 = sh0/sh1
	# q0/q1 = tmp0/tmp1

	# apparently the first 16bytes on the stack are reserved
	addi				a7, a1, 16

    ee.movi.32.a        q2, a6, 2   # a6 = q2[4] | q2[5]
    s16i                a6, a7, 0*2 # a1[0] = q2[4]
    srli                a5, a6, 16
    s16i                a5, a7, 4*2 # a1[4] = q2[5]

    ee.movi.32.a        q3, a6, 2   # a6 = q3[4] | q3[5]
    s16i                a6, a7, 2*2 # a1[2] = q3[4]
    srli                a5, a6, 16
    s16i                a5, a7, 6*2 # a1[6] = q3[5]

    ee.movi.32.a        q2, a6, 3   # a6 = q2[6] | q2[7]
    extui               a5, a6, 0, 16
    neg                 a5, a5
    s16i                a5, a7, 1*2 # a1[1] = -q2[6]
    srli                a5, a6, 16
    neg                 a5, a5
    s16i                a5, a7, 5*2 # a1[5] = -q2[7]

    ee.movi.32.a        q3, a6, 3   # a6 = q3[6] | q3[7]
    extui               a5, a6, 0, 16
    neg                 a5, a5
    s16i                a5, a7, 3*2 # a1[3] = -q2[6]
    srli                a5, a6, 16
    neg                 a5, a5
    s16i                a5, a7, 7*2 # a1[7] = -q2[7]

    ee.vld.128.ip       q2, a7, 0
    ee.vadds.s16        q1, q1, q2

	# Vertical pass
    ee.fft.r2bf.s16     q2, q3, q0, q1, 0
    ee.fft.r2bf.s16     q0, q1, q2, q3, 1
    ee.vzip.16          q0, q1
	# q2/q3 = sv0/sv1
	# q0/q1 = res0/res1

    ee.vzip.16          q0, q1

    # Same as above
    ee.movi.32.a        q2, a6, 2   # a6 = q2[4] | q2[5]
    s16i                a6, a7, 0*2 # a1[0] = q2[4]
    srli                a5, a6, 16
    s16i                a5, a7, 4*2 # a1[4] = q2[5]

    ee.movi.32.a        q3, a6, 2   # a6 = q3[4] | q3[5]
    s16i                a6, a7, 2*2 # a1[2] = q3[4]
    srli                a5, a6, 16
    s16i                a5, a7, 6*2 # a1[6] = q3[5]

    ee.movi.32.a        q2, a6, 3   # a6 = q2[6] | q2[7]
    extui               a5, a6, 0, 16
    neg                 a5, a5
    s16i                a5, a7, 1*2 # a1[1] = -q2[6]
    srli                a5, a6, 16
    neg                 a5, a5
    s16i                a5, a7, 5*2 # a1[5] = -q2[7]

    ee.movi.32.a        q3, a6, 3   # a6 = q3[6] | q3[7]
    extui               a5, a6, 0, 16
    neg                 a5, a5
    s16i                a5, a7, 3*2 # a1[3] = -q2[6]
    srli                a5, a6, 16
    neg                 a5, a5
    s16i                a5, a7, 7*2 # a1[7] = -q2[7]

    ee.vld.128.ip       q2, a7, 0
    ee.vadds.s16        q1, q1, q2

    ee.vunzip.16        q0, q1

    # Output to memory
    ee.vst.128.ip       q0, a4, 16
    ee.vst.128.ip       q1, a4, 0

    retw.n

    .size residual_transform_4x4, .-residual_transform_4x4
