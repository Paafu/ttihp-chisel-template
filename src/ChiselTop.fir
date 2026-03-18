circuit ChiselTop :
  module QuaternionToEuler :
    input clock : Clock
    input reset : Reset
    output io : { flip qw : SInt<32>, flip qx : SInt<32>, flip qy : SInt<32>, flip qz : SInt<32>, roll : SInt<32>, pitch : SInt<32>, yaw : SInt<32>}

    io.roll <= asSInt(UInt<1>("h0")) @[ChiselTop.scala 285:11]
    io.pitch <= asSInt(UInt<1>("h0")) @[ChiselTop.scala 286:12]
    io.yaw <= asSInt(UInt<1>("h0")) @[ChiselTop.scala 287:10]

  module BNO085IMUAdapter :
    input clock : Clock
    input reset : Reset
    output io : { flip bno_quat_w : SInt<16>, flip bno_quat_x : SInt<16>, flip bno_quat_y : SInt<16>, flip bno_quat_z : SInt<16>, pitch : SInt<16>, roll : SInt<16>, yaw : SInt<16>}

    inst quat_to_euler of QuaternionToEuler @[ChiselTop.scala 306:29]
    quat_to_euler.clock <= clock
    quat_to_euler.reset <= reset
    quat_to_euler.io.qw <= io.bno_quat_w @[ChiselTop.scala 307:23]
    quat_to_euler.io.qx <= io.bno_quat_x @[ChiselTop.scala 308:23]
    quat_to_euler.io.qy <= io.bno_quat_y @[ChiselTop.scala 310:23]
    quat_to_euler.io.qz <= io.bno_quat_z @[ChiselTop.scala 311:23]
    node _io_roll_T = bits(quat_to_euler.io.roll, 31, 16) @[ChiselTop.scala 316:35]
    node _io_roll_T_1 = asSInt(_io_roll_T) @[ChiselTop.scala 316:44]
    io.roll <= _io_roll_T_1 @[ChiselTop.scala 316:11]
    node _io_pitch_T = bits(quat_to_euler.io.pitch, 31, 16) @[ChiselTop.scala 317:37]
    node _io_pitch_T_1 = asSInt(_io_pitch_T) @[ChiselTop.scala 317:46]
    io.pitch <= _io_pitch_T_1 @[ChiselTop.scala 317:12]
    node _io_yaw_T = bits(quat_to_euler.io.yaw, 31, 16) @[ChiselTop.scala 318:33]
    node _io_yaw_T_1 = asSInt(_io_yaw_T) @[ChiselTop.scala 318:42]
    io.yaw <= _io_yaw_T_1 @[ChiselTop.scala 318:10]

  module PIDControllerTop :
    input clock : Clock
    input reset : Reset
    output io : { flip pitch_sensor : SInt<8>, flip roll_sensor : SInt<8>, flip yaw_sensor : SInt<8>, flip pitch_desired : SInt<8>, flip roll_desired : SInt<8>, flip yaw_desired : SInt<8>, flip control : UInt<8>, flip data_in : UInt<8>, left_elevon : UInt<8>, right_elevon : UInt<8>, rudder : UInt<8>}

    reg Kp_pitch : UInt<16>, clock with :
      reset => (reset, UInt<16>("h80")) @[ChiselTop.scala 356:25]
    reg Ki_pitch : UInt<16>, clock with :
      reset => (reset, UInt<16>("h1a")) @[ChiselTop.scala 357:25]
    reg Kd_pitch : UInt<16>, clock with :
      reset => (reset, UInt<16>("h3")) @[ChiselTop.scala 358:25]
    reg Kp_roll : UInt<16>, clock with :
      reset => (reset, UInt<16>("h100")) @[ChiselTop.scala 359:24]
    reg Kd_roll : UInt<16>, clock with :
      reset => (reset, UInt<16>("h33")) @[ChiselTop.scala 360:24]
    reg Kp_yaw : UInt<16>, clock with :
      reset => (reset, UInt<16>("h4d")) @[ChiselTop.scala 361:23]
    reg ez_prev : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 364:24]
    reg ey_prev : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 365:24]
    reg integral_ez : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 366:28]
    node _ez_T = sub(io.pitch_desired, io.pitch_sensor) @[ChiselTop.scala 379:26]
    node _ez_T_1 = tail(_ez_T, 1) @[ChiselTop.scala 379:26]
    node ez = asSInt(_ez_T_1) @[ChiselTop.scala 379:26]
    node _ey_T = sub(io.roll_desired, io.roll_sensor) @[ChiselTop.scala 380:25]
    node _ey_T_1 = tail(_ey_T, 1) @[ChiselTop.scala 380:25]
    node ey = asSInt(_ey_T_1) @[ChiselTop.scala 380:25]
    node _er_T = sub(io.yaw_desired, io.yaw_sensor) @[ChiselTop.scala 381:24]
    node _er_T_1 = tail(_er_T, 1) @[ChiselTop.scala 381:24]
    node er = asSInt(_er_T_1) @[ChiselTop.scala 381:24]
    node _integral_ez_T = mul(ez, asSInt(UInt<16>("h100"))) @[ChiselTop.scala 387:38]
    node _integral_ez_T_1 = shr(_integral_ez_T, 8) @[ChiselTop.scala 387:44]
    node _integral_ez_T_2 = add(integral_ez, _integral_ez_T_1) @[ChiselTop.scala 387:30]
    node _integral_ez_T_3 = tail(_integral_ez_T_2, 1) @[ChiselTop.scala 387:30]
    node _integral_ez_T_4 = asSInt(_integral_ez_T_3) @[ChiselTop.scala 387:30]
    integral_ez <= _integral_ez_T_4 @[ChiselTop.scala 387:15]
    node _delta_pitch_T = asSInt(Kp_pitch) @[ChiselTop.scala 390:33]
    node _delta_pitch_T_1 = mul(_delta_pitch_T, ez) @[ChiselTop.scala 390:40]
    node _delta_pitch_T_2 = shr(_delta_pitch_T_1, 8) @[ChiselTop.scala 390:46]
    node _delta_pitch_T_3 = asSInt(Ki_pitch) @[ChiselTop.scala 391:17]
    node _delta_pitch_T_4 = mul(_delta_pitch_T_3, integral_ez) @[ChiselTop.scala 391:24]
    node _delta_pitch_T_5 = shr(_delta_pitch_T_4, 8) @[ChiselTop.scala 391:39]
    node _delta_pitch_T_6 = add(_delta_pitch_T_2, _delta_pitch_T_5) @[ChiselTop.scala 390:60]
    node _delta_pitch_T_7 = tail(_delta_pitch_T_6, 1) @[ChiselTop.scala 390:60]
    node _delta_pitch_T_8 = asSInt(_delta_pitch_T_7) @[ChiselTop.scala 390:60]
    node _delta_pitch_T_9 = asSInt(Kd_pitch) @[ChiselTop.scala 392:17]
    node _delta_pitch_T_10 = sub(ez, ez_prev) @[ChiselTop.scala 392:30]
    node _delta_pitch_T_11 = tail(_delta_pitch_T_10, 1) @[ChiselTop.scala 392:30]
    node _delta_pitch_T_12 = asSInt(_delta_pitch_T_11) @[ChiselTop.scala 392:30]
    node _delta_pitch_T_13 = mul(_delta_pitch_T_9, _delta_pitch_T_12) @[ChiselTop.scala 392:24]
    node _delta_pitch_T_14 = shr(_delta_pitch_T_13, 8) @[ChiselTop.scala 392:42]
    node _delta_pitch_T_15 = add(_delta_pitch_T_8, _delta_pitch_T_14) @[ChiselTop.scala 391:53]
    node _delta_pitch_T_16 = tail(_delta_pitch_T_15, 1) @[ChiselTop.scala 391:53]
    node delta_pitch = asSInt(_delta_pitch_T_16) @[ChiselTop.scala 391:53]
    node _delta_roll_T = asSInt(Kp_roll) @[ChiselTop.scala 393:31]
    node _delta_roll_T_1 = mul(_delta_roll_T, ey) @[ChiselTop.scala 393:38]
    node _delta_roll_T_2 = shr(_delta_roll_T_1, 8) @[ChiselTop.scala 393:44]
    node _delta_roll_T_3 = asSInt(Kd_roll) @[ChiselTop.scala 394:16]
    node _delta_roll_T_4 = sub(ey, ey_prev) @[ChiselTop.scala 394:29]
    node _delta_roll_T_5 = tail(_delta_roll_T_4, 1) @[ChiselTop.scala 394:29]
    node _delta_roll_T_6 = asSInt(_delta_roll_T_5) @[ChiselTop.scala 394:29]
    node _delta_roll_T_7 = mul(_delta_roll_T_3, _delta_roll_T_6) @[ChiselTop.scala 394:23]
    node _delta_roll_T_8 = shr(_delta_roll_T_7, 8) @[ChiselTop.scala 394:41]
    node _delta_roll_T_9 = add(_delta_roll_T_2, _delta_roll_T_8) @[ChiselTop.scala 393:58]
    node _delta_roll_T_10 = tail(_delta_roll_T_9, 1) @[ChiselTop.scala 393:58]
    node delta_roll = asSInt(_delta_roll_T_10) @[ChiselTop.scala 393:58]
    node _delta_yaw_T = asSInt(Kp_yaw) @[ChiselTop.scala 395:29]
    node _delta_yaw_T_1 = mul(_delta_yaw_T, er) @[ChiselTop.scala 395:36]
    node delta_yaw = shr(_delta_yaw_T_1, 8) @[ChiselTop.scala 395:42]
    ez_prev <= ez @[ChiselTop.scala 398:11]
    ey_prev <= ey @[ChiselTop.scala 399:11]
    node _elevon_left_T = add(delta_pitch, delta_roll) @[ChiselTop.scala 402:33]
    node _elevon_left_T_1 = tail(_elevon_left_T, 1) @[ChiselTop.scala 402:33]
    node elevon_left = asSInt(_elevon_left_T_1) @[ChiselTop.scala 402:33]
    node _elevon_right_T = sub(delta_pitch, delta_roll) @[ChiselTop.scala 403:34]
    node _elevon_right_T_1 = tail(_elevon_right_T, 1) @[ChiselTop.scala 403:34]
    node elevon_right = asSInt(_elevon_right_T_1) @[ChiselTop.scala 403:34]
    node _left_elevon_pwm_T = asUInt(elevon_left) @[ChiselTop.scala 406:39]
    node _left_elevon_pwm_T_1 = add(_left_elevon_pwm_T, UInt<9>("h100")) @[ChiselTop.scala 406:46]
    node _left_elevon_pwm_T_2 = tail(_left_elevon_pwm_T_1, 1) @[ChiselTop.scala 406:46]
    node left_elevon_pwm = bits(_left_elevon_pwm_T_2, 7, 0) @[ChiselTop.scala 406:54]
    node _right_elevon_pwm_T = asUInt(elevon_right) @[ChiselTop.scala 407:41]
    node _right_elevon_pwm_T_1 = add(_right_elevon_pwm_T, UInt<9>("h100")) @[ChiselTop.scala 407:48]
    node _right_elevon_pwm_T_2 = tail(_right_elevon_pwm_T_1, 1) @[ChiselTop.scala 407:48]
    node right_elevon_pwm = bits(_right_elevon_pwm_T_2, 7, 0) @[ChiselTop.scala 407:56]
    node _rudder_pwm_T = asUInt(delta_yaw) @[ChiselTop.scala 408:32]
    node _rudder_pwm_T_1 = add(_rudder_pwm_T, UInt<9>("h100")) @[ChiselTop.scala 408:39]
    node _rudder_pwm_T_2 = tail(_rudder_pwm_T_1, 1) @[ChiselTop.scala 408:39]
    node rudder_pwm = bits(_rudder_pwm_T_2, 7, 0) @[ChiselTop.scala 408:47]
    io.left_elevon <= left_elevon_pwm @[ChiselTop.scala 411:18]
    io.right_elevon <= right_elevon_pwm @[ChiselTop.scala 412:19]
    io.rudder <= rudder_pwm @[ChiselTop.scala 413:13]
    node reg_select = bits(io.control, 2, 0) @[ChiselTop.scala 500:30]
    node _load_high_T = bits(io.control, 3, 3) @[ChiselTop.scala 501:29]
    node load_high = eq(_load_high_T, UInt<1>("h0")) @[ChiselTop.scala 501:33]
    when load_high : @[ChiselTop.scala 503:19]
      node _T = eq(UInt<1>("h0"), reg_select) @[ChiselTop.scala 504:24]
      when _T : @[ChiselTop.scala 504:24]
        node _Kp_pitch_T = bits(Kp_pitch, 7, 0) @[ChiselTop.scala 505:53]
        node _Kp_pitch_T_1 = cat(io.data_in, _Kp_pitch_T) @[Cat.scala 33:92]
        Kp_pitch <= _Kp_pitch_T_1 @[ChiselTop.scala 505:26]
      else :
        node _T_1 = eq(UInt<1>("h1"), reg_select) @[ChiselTop.scala 504:24]
        when _T_1 : @[ChiselTop.scala 504:24]
          node _Ki_pitch_T = bits(Ki_pitch, 7, 0) @[ChiselTop.scala 506:53]
          node _Ki_pitch_T_1 = cat(io.data_in, _Ki_pitch_T) @[Cat.scala 33:92]
          Ki_pitch <= _Ki_pitch_T_1 @[ChiselTop.scala 506:26]
        else :
          node _T_2 = eq(UInt<2>("h2"), reg_select) @[ChiselTop.scala 504:24]
          when _T_2 : @[ChiselTop.scala 504:24]
            node _Kd_pitch_T = bits(Kd_pitch, 7, 0) @[ChiselTop.scala 507:53]
            node _Kd_pitch_T_1 = cat(io.data_in, _Kd_pitch_T) @[Cat.scala 33:92]
            Kd_pitch <= _Kd_pitch_T_1 @[ChiselTop.scala 507:26]
          else :
            node _T_3 = eq(UInt<2>("h3"), reg_select) @[ChiselTop.scala 504:24]
            when _T_3 : @[ChiselTop.scala 504:24]
              node _Kp_roll_T = bits(Kp_roll, 7, 0) @[ChiselTop.scala 508:51]
              node _Kp_roll_T_1 = cat(io.data_in, _Kp_roll_T) @[Cat.scala 33:92]
              Kp_roll <= _Kp_roll_T_1 @[ChiselTop.scala 508:25]
            else :
              node _T_4 = eq(UInt<3>("h4"), reg_select) @[ChiselTop.scala 504:24]
              when _T_4 : @[ChiselTop.scala 504:24]
                node _Kd_roll_T = bits(Kd_roll, 7, 0) @[ChiselTop.scala 509:51]
                node _Kd_roll_T_1 = cat(io.data_in, _Kd_roll_T) @[Cat.scala 33:92]
                Kd_roll <= _Kd_roll_T_1 @[ChiselTop.scala 509:25]
              else :
                node _T_5 = eq(UInt<3>("h5"), reg_select) @[ChiselTop.scala 504:24]
                when _T_5 : @[ChiselTop.scala 504:24]
                  node _Kp_yaw_T = bits(Kp_yaw, 7, 0) @[ChiselTop.scala 510:49]
                  node _Kp_yaw_T_1 = cat(io.data_in, _Kp_yaw_T) @[Cat.scala 33:92]
                  Kp_yaw <= _Kp_yaw_T_1 @[ChiselTop.scala 510:24]
    else :
      node _T_6 = eq(UInt<1>("h0"), reg_select) @[ChiselTop.scala 513:24]
      when _T_6 : @[ChiselTop.scala 513:24]
        node _Kp_pitch_T_2 = bits(Kp_pitch, 15, 8) @[ChiselTop.scala 514:41]
        node _Kp_pitch_T_3 = cat(_Kp_pitch_T_2, io.data_in) @[Cat.scala 33:92]
        Kp_pitch <= _Kp_pitch_T_3 @[ChiselTop.scala 514:26]
      else :
        node _T_7 = eq(UInt<1>("h1"), reg_select) @[ChiselTop.scala 513:24]
        when _T_7 : @[ChiselTop.scala 513:24]
          node _Ki_pitch_T_2 = bits(Ki_pitch, 15, 8) @[ChiselTop.scala 515:41]
          node _Ki_pitch_T_3 = cat(_Ki_pitch_T_2, io.data_in) @[Cat.scala 33:92]
          Ki_pitch <= _Ki_pitch_T_3 @[ChiselTop.scala 515:26]
        else :
          node _T_8 = eq(UInt<2>("h2"), reg_select) @[ChiselTop.scala 513:24]
          when _T_8 : @[ChiselTop.scala 513:24]
            node _Kd_pitch_T_2 = bits(Kd_pitch, 15, 8) @[ChiselTop.scala 516:41]
            node _Kd_pitch_T_3 = cat(_Kd_pitch_T_2, io.data_in) @[Cat.scala 33:92]
            Kd_pitch <= _Kd_pitch_T_3 @[ChiselTop.scala 516:26]
          else :
            node _T_9 = eq(UInt<2>("h3"), reg_select) @[ChiselTop.scala 513:24]
            when _T_9 : @[ChiselTop.scala 513:24]
              node _Kp_roll_T_2 = bits(Kp_roll, 15, 8) @[ChiselTop.scala 517:39]
              node _Kp_roll_T_3 = cat(_Kp_roll_T_2, io.data_in) @[Cat.scala 33:92]
              Kp_roll <= _Kp_roll_T_3 @[ChiselTop.scala 517:25]
            else :
              node _T_10 = eq(UInt<3>("h4"), reg_select) @[ChiselTop.scala 513:24]
              when _T_10 : @[ChiselTop.scala 513:24]
                node _Kd_roll_T_2 = bits(Kd_roll, 15, 8) @[ChiselTop.scala 518:39]
                node _Kd_roll_T_3 = cat(_Kd_roll_T_2, io.data_in) @[Cat.scala 33:92]
                Kd_roll <= _Kd_roll_T_3 @[ChiselTop.scala 518:25]
              else :
                node _T_11 = eq(UInt<3>("h5"), reg_select) @[ChiselTop.scala 513:24]
                when _T_11 : @[ChiselTop.scala 513:24]
                  node _Kp_yaw_T_2 = bits(Kp_yaw, 15, 8) @[ChiselTop.scala 519:37]
                  node _Kp_yaw_T_3 = cat(_Kp_yaw_T_2, io.data_in) @[Cat.scala 33:92]
                  Kp_yaw <= _Kp_yaw_T_3 @[ChiselTop.scala 519:24]


  module DS1050PWMAdapter :
    input clock : Clock
    input reset : Reset
    output io : { flip pid_pwm : UInt<16>, ds_pwm : UInt<5>}

    node _io_ds_pwm_T = shr(io.pid_pwm, 11) @[ChiselTop.scala 332:27]
    io.ds_pwm <= _io_ds_pwm_T @[ChiselTop.scala 332:13]

  module DS1050PWMAdapter_1 :
    input clock : Clock
    input reset : Reset
    output io : { flip pid_pwm : UInt<16>, ds_pwm : UInt<5>}

    node _io_ds_pwm_T = shr(io.pid_pwm, 11) @[ChiselTop.scala 332:27]
    io.ds_pwm <= _io_ds_pwm_T @[ChiselTop.scala 332:13]

  module DS1050PWMAdapter_2 :
    input clock : Clock
    input reset : Reset
    output io : { flip pid_pwm : UInt<16>, ds_pwm : UInt<5>}

    node _io_ds_pwm_T = shr(io.pid_pwm, 11) @[ChiselTop.scala 332:27]
    io.ds_pwm <= _io_ds_pwm_T @[ChiselTop.scala 332:13]

  module DataCollector :
    input clock : Clock
    input reset : Reset
    output io : { flip ui_in : UInt<8>, uo_out : UInt<8>, flip uio_in : UInt<8>, uio_out : UInt<8>, uio_oe : UInt<8>, word_out : UInt<16>, word_valid : UInt<1>}

    reg state : UInt<2>, clock with :
      reset => (reset, UInt<2>("h0")) @[ChiselTop.scala 540:22]
    reg dataReg : UInt<16>, clock with :
      reset => (reset, UInt<16>("h0")) @[ChiselTop.scala 543:24]
    reg byteCount : UInt<1>, clock with :
      reset => (reset, UInt<1>("h0")) @[ChiselTop.scala 544:26]
    reg word_out_reg : UInt<16>, clock with :
      reset => (reset, UInt<16>("h0")) @[ChiselTop.scala 545:29]
    reg word_valid_reg : UInt<1>, clock with :
      reset => (reset, UInt<1>("h0")) @[ChiselTop.scala 546:31]
    io.uo_out <= UInt<1>("h0") @[ChiselTop.scala 549:13]
    io.uio_out <= UInt<1>("h0") @[ChiselTop.scala 550:14]
    io.uio_oe <= UInt<1>("h0") @[ChiselTop.scala 551:13]
    io.word_out <= word_out_reg @[ChiselTop.scala 552:15]
    io.word_valid <= word_valid_reg @[ChiselTop.scala 553:17]
    node _T = eq(UInt<2>("h0"), state) @[ChiselTop.scala 555:17]
    when _T : @[ChiselTop.scala 555:17]
      node _T_1 = neq(io.ui_in, UInt<1>("h0")) @[ChiselTop.scala 558:21]
      when _T_1 : @[ChiselTop.scala 558:30]
        state <= UInt<2>("h1") @[ChiselTop.scala 559:15]
        byteCount <= UInt<1>("h0") @[ChiselTop.scala 560:19]
        node _dataReg_T = cat(io.ui_in, UInt<8>("h0")) @[Cat.scala 33:92]
        dataReg <= _dataReg_T @[ChiselTop.scala 562:17]
        word_valid_reg <= UInt<1>("h0") @[ChiselTop.scala 563:24]
    else :
      node _T_2 = eq(UInt<2>("h1"), state) @[ChiselTop.scala 555:17]
      when _T_2 : @[ChiselTop.scala 555:17]
        node _T_3 = eq(byteCount, UInt<1>("h0")) @[ChiselTop.scala 567:22]
        when _T_3 : @[ChiselTop.scala 567:31]
          node _dataReg_T_1 = bits(dataReg, 15, 8) @[ChiselTop.scala 569:31]
          node _dataReg_T_2 = cat(_dataReg_T_1, io.ui_in) @[Cat.scala 33:92]
          dataReg <= _dataReg_T_2 @[ChiselTop.scala 569:17]
          byteCount <= UInt<1>("h1") @[ChiselTop.scala 570:19]
          state <= UInt<2>("h2") @[ChiselTop.scala 571:15]
      else :
        node _T_4 = eq(UInt<2>("h2"), state) @[ChiselTop.scala 555:17]
        when _T_4 : @[ChiselTop.scala 555:17]
          node _io_uo_out_T = bits(dataReg, 7, 0) @[ChiselTop.scala 576:27]
          io.uo_out <= _io_uo_out_T @[ChiselTop.scala 576:17]
          word_out_reg <= dataReg @[ChiselTop.scala 578:20]
          word_valid_reg <= UInt<1>("h1") @[ChiselTop.scala 579:22]
          state <= UInt<2>("h3") @[ChiselTop.scala 580:13]
        else :
          node _T_5 = eq(UInt<2>("h3"), state) @[ChiselTop.scala 555:17]
          when _T_5 : @[ChiselTop.scala 555:17]
            node _io_uio_out_T = bits(dataReg, 15, 8) @[ChiselTop.scala 583:28]
            io.uio_out <= _io_uio_out_T @[ChiselTop.scala 583:18]
            node _io_uio_oe_T = mux(UInt<1>("h1"), UInt<8>("hff"), UInt<8>("h0")) @[Bitwise.scala 77:12]
            io.uio_oe <= _io_uio_oe_T @[ChiselTop.scala 584:17]
            state <= UInt<2>("h0") @[ChiselTop.scala 586:13]
            word_valid_reg <= UInt<1>("h0") @[ChiselTop.scala 587:22]


  module ChiselTop :
    input clock : Clock
    input reset : UInt<1>
    output io : { flip ui_in : UInt<8>, uo_out : UInt<8>, flip uio_in : UInt<8>, uio_out : UInt<8>, uio_oe : UInt<8>}

    inst imu_adapter of BNO085IMUAdapter @[ChiselTop.scala 48:27]
    imu_adapter.clock <= clock
    imu_adapter.reset <= reset
    inst pid of PIDControllerTop @[ChiselTop.scala 49:19]
    pid.clock <= clock
    pid.reset <= reset
    inst ds_left of DS1050PWMAdapter @[ChiselTop.scala 50:23]
    ds_left.clock <= clock
    ds_left.reset <= reset
    inst ds_right of DS1050PWMAdapter_1 @[ChiselTop.scala 51:24]
    ds_right.clock <= clock
    ds_right.reset <= reset
    inst ds_rudder of DS1050PWMAdapter_2 @[ChiselTop.scala 52:25]
    ds_rudder.clock <= clock
    ds_rudder.reset <= reset
    inst dataCollector of DataCollector @[ChiselTop.scala 55:29]
    dataCollector.clock <= clock
    dataCollector.reset <= reset
    dataCollector.io.ui_in <= io.ui_in @[ChiselTop.scala 58:26]
    dataCollector.io.uio_in <= io.uio_in @[ChiselTop.scala 59:27]
    io.uio_out <= dataCollector.io.uio_out @[ChiselTop.scala 65:14]
    io.uio_oe <= dataCollector.io.uio_oe @[ChiselTop.scala 66:13]
    reg bno_quat_w_reg : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 72:31]
    reg bno_quat_x_reg : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 73:31]
    reg bno_quat_y_reg : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 74:31]
    reg bno_quat_z_reg : SInt<16>, clock with :
      reset => (reset, asSInt(UInt<16>("h0"))) @[ChiselTop.scala 75:31]
    reg pitch_desired_reg : SInt<8>, clock with :
      reset => (reset, asSInt(UInt<8>("h0"))) @[ChiselTop.scala 78:34]
    reg roll_desired_reg : SInt<8>, clock with :
      reset => (reset, asSInt(UInt<8>("h0"))) @[ChiselTop.scala 79:33]
    reg yaw_desired_reg : SInt<8>, clock with :
      reset => (reset, asSInt(UInt<8>("h0"))) @[ChiselTop.scala 80:32]
    reg control_reg : UInt<8>, clock with :
      reset => (reset, UInt<8>("h0")) @[ChiselTop.scala 81:28]
    reg data_in_reg : UInt<8>, clock with :
      reset => (reset, UInt<8>("h0")) @[ChiselTop.scala 82:28]
    reg pending_target : UInt<8>, clock with :
      reset => (reset, UInt<8>("h0")) @[ChiselTop.scala 95:31]
    reg expecting_data : UInt<1>, clock with :
      reset => (reset, UInt<1>("h0")) @[ChiselTop.scala 96:31]
    when dataCollector.io.word_valid : @[ChiselTop.scala 97:37]
      node _T = eq(expecting_data, UInt<1>("h0")) @[ChiselTop.scala 99:10]
      when _T : @[ChiselTop.scala 99:27]
        node _T_1 = bits(dataCollector.io.word_out, 15, 8) @[ChiselTop.scala 100:13]
        node _T_2 = eq(_T_1, UInt<8>("hff")) @[ChiselTop.scala 100:20]
        when _T_2 : @[ChiselTop.scala 100:32]
          node _pending_target_T = bits(dataCollector.io.word_out, 7, 0) @[ChiselTop.scala 101:28]
          pending_target <= _pending_target_T @[ChiselTop.scala 101:24]
          expecting_data <= UInt<1>("h1") @[ChiselTop.scala 102:24]
      else :
        node _T_3 = eq(UInt<1>("h0"), pending_target) @[ChiselTop.scala 106:30]
        when _T_3 : @[ChiselTop.scala 106:30]
          node _bno_quat_w_reg_T = asSInt(dataCollector.io.word_out) @[ChiselTop.scala 107:39]
          bno_quat_w_reg <= _bno_quat_w_reg_T @[ChiselTop.scala 107:34]
        else :
          node _T_4 = eq(UInt<1>("h1"), pending_target) @[ChiselTop.scala 106:30]
          when _T_4 : @[ChiselTop.scala 106:30]
            node _bno_quat_x_reg_T = asSInt(dataCollector.io.word_out) @[ChiselTop.scala 108:39]
            bno_quat_x_reg <= _bno_quat_x_reg_T @[ChiselTop.scala 108:34]
          else :
            node _T_5 = eq(UInt<2>("h2"), pending_target) @[ChiselTop.scala 106:30]
            when _T_5 : @[ChiselTop.scala 106:30]
              node _bno_quat_y_reg_T = asSInt(dataCollector.io.word_out) @[ChiselTop.scala 109:39]
              bno_quat_y_reg <= _bno_quat_y_reg_T @[ChiselTop.scala 109:34]
            else :
              node _T_6 = eq(UInt<2>("h3"), pending_target) @[ChiselTop.scala 106:30]
              when _T_6 : @[ChiselTop.scala 106:30]
                node _bno_quat_z_reg_T = asSInt(dataCollector.io.word_out) @[ChiselTop.scala 110:39]
                bno_quat_z_reg <= _bno_quat_z_reg_T @[ChiselTop.scala 110:34]
              else :
                node _T_7 = eq(UInt<5>("h10"), pending_target) @[ChiselTop.scala 106:30]
                when _T_7 : @[ChiselTop.scala 106:30]
                  node _pitch_desired_reg_T = bits(dataCollector.io.word_out, 7, 0) @[ChiselTop.scala 111:44]
                  node _pitch_desired_reg_T_1 = asSInt(_pitch_desired_reg_T) @[ChiselTop.scala 111:50]
                  pitch_desired_reg <= _pitch_desired_reg_T_1 @[ChiselTop.scala 111:40]
                else :
                  node _T_8 = eq(UInt<5>("h11"), pending_target) @[ChiselTop.scala 106:30]
                  when _T_8 : @[ChiselTop.scala 106:30]
                    node _roll_desired_reg_T = bits(dataCollector.io.word_out, 7, 0) @[ChiselTop.scala 112:43]
                    node _roll_desired_reg_T_1 = asSInt(_roll_desired_reg_T) @[ChiselTop.scala 112:49]
                    roll_desired_reg <= _roll_desired_reg_T_1 @[ChiselTop.scala 112:39]
                  else :
                    node _T_9 = eq(UInt<5>("h12"), pending_target) @[ChiselTop.scala 106:30]
                    when _T_9 : @[ChiselTop.scala 106:30]
                      node _yaw_desired_reg_T = bits(dataCollector.io.word_out, 7, 0) @[ChiselTop.scala 113:42]
                      node _yaw_desired_reg_T_1 = asSInt(_yaw_desired_reg_T) @[ChiselTop.scala 113:48]
                      yaw_desired_reg <= _yaw_desired_reg_T_1 @[ChiselTop.scala 113:38]
                    else :
                      node _T_10 = eq(UInt<6>("h20"), pending_target) @[ChiselTop.scala 106:30]
                      when _T_10 : @[ChiselTop.scala 106:30]
                        node _control_reg_T = bits(dataCollector.io.word_out, 7, 0) @[ChiselTop.scala 114:38]
                        control_reg <= _control_reg_T @[ChiselTop.scala 114:34]
                      else :
                        node _T_11 = eq(UInt<6>("h21"), pending_target) @[ChiselTop.scala 106:30]
                        when _T_11 : @[ChiselTop.scala 106:30]
                          node _data_in_reg_T = bits(dataCollector.io.word_out, 7, 0) @[ChiselTop.scala 115:38]
                          data_in_reg <= _data_in_reg_T @[ChiselTop.scala 115:34]
        expecting_data <= UInt<1>("h0") @[ChiselTop.scala 117:22]
    imu_adapter.io.bno_quat_w <= bno_quat_w_reg @[ChiselTop.scala 122:29]
    imu_adapter.io.bno_quat_x <= bno_quat_x_reg @[ChiselTop.scala 123:29]
    imu_adapter.io.bno_quat_y <= bno_quat_y_reg @[ChiselTop.scala 124:29]
    imu_adapter.io.bno_quat_z <= bno_quat_z_reg @[ChiselTop.scala 125:29]
    node _pid_io_pitch_sensor_T = bits(imu_adapter.io.pitch, 15, 8) @[ChiselTop.scala 130:46]
    node _pid_io_pitch_sensor_T_1 = asSInt(_pid_io_pitch_sensor_T) @[ChiselTop.scala 130:54]
    pid.io.pitch_sensor <= _pid_io_pitch_sensor_T_1 @[ChiselTop.scala 130:23]
    node _pid_io_roll_sensor_T = bits(imu_adapter.io.roll, 15, 8) @[ChiselTop.scala 131:44]
    node _pid_io_roll_sensor_T_1 = asSInt(_pid_io_roll_sensor_T) @[ChiselTop.scala 131:52]
    pid.io.roll_sensor <= _pid_io_roll_sensor_T_1 @[ChiselTop.scala 131:22]
    node _pid_io_yaw_sensor_T = bits(imu_adapter.io.yaw, 15, 8) @[ChiselTop.scala 132:42]
    node _pid_io_yaw_sensor_T_1 = asSInt(_pid_io_yaw_sensor_T) @[ChiselTop.scala 132:50]
    pid.io.yaw_sensor <= _pid_io_yaw_sensor_T_1 @[ChiselTop.scala 132:21]
    pid.io.pitch_desired <= pitch_desired_reg @[ChiselTop.scala 135:24]
    pid.io.roll_desired <= roll_desired_reg @[ChiselTop.scala 136:23]
    pid.io.yaw_desired <= yaw_desired_reg @[ChiselTop.scala 137:22]
    pid.io.control <= control_reg @[ChiselTop.scala 140:18]
    pid.io.data_in <= data_in_reg @[ChiselTop.scala 141:18]
    wire left_elevon_wire : UInt<8> @[ChiselTop.scala 144:30]
    wire right_elevon_wire : UInt<8> @[ChiselTop.scala 145:31]
    wire rudder_wire : UInt<8> @[ChiselTop.scala 146:25]
    left_elevon_wire <= pid.io.left_elevon @[ChiselTop.scala 147:20]
    right_elevon_wire <= pid.io.right_elevon @[ChiselTop.scala 148:21]
    rudder_wire <= pid.io.rudder @[ChiselTop.scala 149:15]
    node _ds_left_io_pid_pwm_T = cat(UInt<8>("h0"), left_elevon_wire) @[Cat.scala 33:92]
    ds_left.io.pid_pwm <= _ds_left_io_pid_pwm_T @[ChiselTop.scala 152:22]
    node _ds_right_io_pid_pwm_T = cat(UInt<8>("h0"), right_elevon_wire) @[Cat.scala 33:92]
    ds_right.io.pid_pwm <= _ds_right_io_pid_pwm_T @[ChiselTop.scala 153:23]
    node _ds_rudder_io_pid_pwm_T = cat(UInt<8>("h0"), rudder_wire) @[Cat.scala 33:92]
    ds_rudder.io.pid_pwm <= _ds_rudder_io_pid_pwm_T @[ChiselTop.scala 154:24]
    reg pwm_cycle : UInt<2>, clock with :
      reset => (reset, UInt<2>("h0")) @[ChiselTop.scala 181:26]
    node _pwm_cycle_T = add(pwm_cycle, UInt<1>("h1")) @[ChiselTop.scala 182:26]
    node _pwm_cycle_T_1 = tail(_pwm_cycle_T, 1) @[ChiselTop.scala 182:26]
    pwm_cycle <= _pwm_cycle_T_1 @[ChiselTop.scala 182:13]
    node tag_left = cat(UInt<3>("h1"), ds_left.io.ds_pwm) @[Cat.scala 33:92]
    node tag_right = cat(UInt<3>("h2"), ds_right.io.ds_pwm) @[Cat.scala 33:92]
    node tag_rudder = cat(UInt<3>("h3"), ds_rudder.io.ds_pwm) @[Cat.scala 33:92]
    wire packed_uo : UInt @[ChiselTop.scala 190:30]
    packed_uo <= dataCollector.io.uo_out @[ChiselTop.scala 190:30]
    node _T_12 = eq(dataCollector.io.word_valid, UInt<1>("h0")) @[ChiselTop.scala 193:8]
    when _T_12 : @[ChiselTop.scala 193:38]
      node _packed_uo_T = eq(UInt<1>("h0"), pwm_cycle) @[Mux.scala 81:61]
      node _packed_uo_T_1 = mux(_packed_uo_T, tag_left, UInt<8>("h0")) @[Mux.scala 81:58]
      node _packed_uo_T_2 = eq(UInt<1>("h1"), pwm_cycle) @[Mux.scala 81:61]
      node _packed_uo_T_3 = mux(_packed_uo_T_2, tag_right, _packed_uo_T_1) @[Mux.scala 81:58]
      node _packed_uo_T_4 = eq(UInt<2>("h2"), pwm_cycle) @[Mux.scala 81:61]
      node _packed_uo_T_5 = mux(_packed_uo_T_4, tag_rudder, _packed_uo_T_3) @[Mux.scala 81:58]
      packed_uo <= _packed_uo_T_5 @[ChiselTop.scala 194:15]
    io.uo_out <= packed_uo @[ChiselTop.scala 202:13]

