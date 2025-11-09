package hardfloat

import chisel3._
import chisel3.util._


class SrtTable extends Module {
  val io = IO(new Bundle() {
    val d = Input(UInt(3.W))
    val y = Input(UInt(8.W))
    val q = Output(SInt(3.W))
  })
  val qSelTable = Array(
    Array(12, 4, -4, -13),
    Array(14, 4, -5, -14),
    Array(16, 4, -6, -16),
    Array(16, 4, -6, -17),
    Array(18, 6, -6, -18),
    Array(20, 6, -8, -20),
    Array(20, 8, -8, -22),
    Array(24, 8, -8, -23)
  ).map(_.map(_ * 2))

  var ge = Map[Int, Bool]()
  for(row <- qSelTable){
    for(k <- row){
      if(!ge.contains(k)) ge = ge + (k -> (io.y.asSInt >= k.S(8.W)))
    }
  }
  io.q := MuxLookup(io.d, 0.S)(
    qSelTable.map(x =>
      MuxCase((-2).S(3.W), Seq(
        ge(x(0)) -> 2.S(3.W),
        ge(x(1)) -> 1.S(3.W),
        ge(x(2)) -> 0.S(3.W),
        ge(x(3)) -> (-1).S(3.W)
      ))
    ).zipWithIndex.map({case(v, i) => i.U -> v})
  )
}

class OnTheFlyConv(len: Int) extends Module {
  val io = IO(new Bundle() {
    val resetSqrt = Input(Bool())
    val resetDiv = Input(Bool())
    val enable = Input(Bool())
    val qi = Input(SInt(3.W))
    val QM = Output(UInt(len.W))
    val Q = Output(UInt(len.W))
    val F = Output(UInt(len.W))
  })
  val Q, QM = Reg(UInt(len.W))

  /**  FGen:
    *  use additional regs to avoid
    *  big width shifter since FGen is in cirtical path
    */
  val mask = Reg(SInt(len.W))
  val b_111, b_1100 = Reg(UInt(len.W))
  when(io.resetSqrt){
    mask := Cat("b1".U(1.W), 0.U((len-1).W)).asSInt
    b_111 := "b111".U(3.W) << (len-5)
    b_1100 := "b1100".U(4.W) << (len-5)
  }.elsewhen(io.enable){
    mask := mask >> 2
    b_111 := b_111 >> 2
    b_1100 := b_1100 >> 2
  }
  val b_00, b_01, b_10, b_11 = Reg(UInt((len-3).W))
  b_00 := 0.U
  when(io.resetDiv || io.resetSqrt){
    b_01 := Cat("b01".U(2.W), 0.U((len-5).W))
    b_10 := Cat("b10".U(2.W), 0.U((len-5).W))
    b_11 := Cat("b11".U(2.W), 0.U((len-5).W))
  }.elsewhen(io.enable){
    b_01 := b_01 >> 2
    b_10 := b_10 >> 2
    b_11 := b_11 >> 2
  }

  val negQ = ~Q
  val sqrtToCsaMap = Seq(
    1 -> (negQ, b_111),
    2 -> (negQ, b_1100),
    -1 -> (QM, b_111),
    -2 -> (QM, b_1100)
  ).map(
    m => m._1.S(3.W).asUInt ->
      ( ((m._2._1 << Mux(io.qi(0), 1.U, 2.U)).asUInt & (mask >> io.qi(0)).asUInt) | m._2._2 )
  )
  val sqrtToCsa = MuxLookup(io.qi.asUInt, 0.U)(sqrtToCsaMap)

  val Q_load_00 = Q | b_00
  val Q_load_01 = Q | b_01
  val Q_load_10 = Q | b_10
  val QM_load_01 = QM | b_01
  val QM_load_10 = QM | b_10
  val QM_load_11 = QM | b_11

  when(io.resetSqrt){
    Q := Cat(1.U(3.W), 0.U((len-3).W))
    QM := 0.U
  }.elsewhen(io.resetDiv){
    Q := 0.U
    QM := 0.U
  }.elsewhen(io.enable){
    val QConvMap = Seq(
      0 -> Q_load_00,
      1 -> Q_load_01,
      2 -> Q_load_10,
      -1 -> QM_load_11,
      -2 -> QM_load_10
    ).map(m => m._1.S(3.W).asUInt -> m._2)
    val QMConvMap = Seq(
      0 -> QM_load_11,
      1 -> Q_load_00,
      2 -> Q_load_01,
      -1 -> QM_load_10,
      -2 -> QM_load_01
    ).map(m => m._1.S(3.W).asUInt -> m._2)
    Q := MuxLookup(io.qi.asUInt, 0.U)(QConvMap)
    QM := MuxLookup(io.qi.asUInt, 0.U)(QMConvMap)
  }

  io.F := sqrtToCsa
  io.QM := QM
  io.Q := Q
}

class SigDivSqrt_srt4(len: Int) extends Module {
  val io = IO(new Bundle() {
    val kill = Input(Bool())
    val in = Flipped(DecoupledIO(new Bundle() {
      val sigA, sigB = UInt(len.W)
      val isDiv = Bool()
      val dsCycles = UInt(log2Up(len).W)
    }))
    val out = DecoupledIO(new Bundle() {
      val quotient = UInt(len.W)
      val isZeroRem = Bool()
    })
  })

  val (a, b) = (io.in.bits.sigA, io.in.bits.sigB)
  val isDiv = io.in.bits.isDiv
  val isDivReg = RegEnable(isDiv, io.in.fire)
  val divisor = RegEnable(b, io.in.fire)

  val s_idle :: s_recurrence :: s_recovery :: s_finish :: Nil = Enum(4)
  val state = RegInit(s_idle)
  val cnt_next = Wire(UInt(log2Up((len+1)/2).W))
  val cnt = RegEnable(cnt_next, state===s_idle || state===s_recurrence)
  cnt_next := Mux(state === s_idle, io.in.bits.dsCycles, cnt - 1.U)


  val firstCycle = RegNext(io.in.fire)

  switch(state){
    is(s_idle){
      when(io.in.fire){ state := s_recurrence }
    }
    is(s_recurrence){
      when(cnt_next === 0.U){ state := s_recovery }
    }
    is(s_recovery){
      state := s_finish
    }
    is(s_finish){
      when(io.out.fire){ state := s_idle }
    }
  }
  when(io.kill){ state := s_idle }

  val ws, wc = Reg(UInt((len+4).W))


  val table = Module(new SrtTable)
  val conv = Module(new OnTheFlyConv(len+3))
  val csa = Module(new CSA3_2(len+4))

  // partial square root
  val S = conv.io.Q >> 2
  val s0 :: s1 :: s2 :: s3 :: s4 :: Nil =  S(len-2, len-6).asBools.reverse
  val sqrt_d = Mux(firstCycle, "b101".U(3.W), Mux(s0, "b111".U(3.W), Cat(s2, s3, s4)))
  val div_d = divisor(len-2, len-4)
  val sqrt_y = ws(len+3, len-4) + wc(len+3, len-4)
  val div_y = ws(len+2, len-5) + wc(len+2, len-5)

  table.io.d := Mux(isDivReg, div_d, sqrt_d)
  table.io.y := Mux(isDivReg, div_y, sqrt_y)

  conv.io.resetSqrt := io.in.fire && !isDiv
  conv.io.resetDiv := io.in.fire && isDiv
  conv.io.enable := state===s_recurrence
  conv.io.qi := table.io.q

  val dx1, dx2, neg_dx1, neg_dx2 = Wire(UInt((len+4).W))
  dx1 := divisor
  dx2 := divisor << 1
  neg_dx1 := ~dx1
  neg_dx2 := neg_dx1 << 1

  val divCsaIn = MuxLookup(table.io.q.asUInt, 0.U)(Seq(
    -1 -> dx1,
    -2 -> dx2,
    1 -> neg_dx1,
    2 -> neg_dx2
  ).map(m => m._1.S(3.W).asUInt -> m._2))

  csa.io.in(0) := ws
  csa.io.in(1) := Mux(isDivReg & !table.io.q(2),  wc | table.io.q(1, 0), wc)
  csa.io.in(2) := Mux(isDivReg, divCsaIn, conv.io.F)

  val divWsInit =  a
  val sqrtWsInit = Cat( Cat(0.U(2.W), a) - Cat(1.U(2.W), 0.U(len.W)), 0.U(2.W))

  when(io.in.fire){
    ws := Mux(isDiv, divWsInit, sqrtWsInit)
    wc := 0.U
  }.elsewhen(state === s_recurrence){
    ws := Mux(cnt_next === 0.U, csa.io.out(0), csa.io.out(0) << 2)
    wc := Mux(cnt_next === 0.U, csa.io.out(1) << 1, csa.io.out(1) << 3)
  }

  val rem = ws + wc

  /** Remainder format:
    * Sqrt:
    * s s x x. x x x ... x
    * Div:
    * s s s x. x x x ... x
    */
  val remSignReg = RegEnable(rem.head(1).asBool, state===s_recovery)
  val isZeroRemReg = RegEnable(rem===0.U, state===s_recovery)

  io.in.ready := state === s_idle
  io.out.valid := state === s_finish
  io.out.bits.quotient := Mux(remSignReg, conv.io.QM, conv.io.Q) >> Cat(!isDivReg, 0.U(1.W))
  io.out.bits.isZeroRem := isZeroRemReg

}

class DivSqrtRawFN_srt4(expWidth: Int, sigWidth: Int) extends Module {
  val io = IO(new Bundle {
    /*--------------------------------------------------------------------
    *--------------------------------------------------------------------*/
    val inReady = Output(Bool())
    val inValid = Input(Bool())
    val sqrtOp = Input(Bool())
    val a = Input(new RawFloat(expWidth, sigWidth))
    val b = Input(new RawFloat(expWidth, sigWidth))
    val roundingMode = Input(UInt(3.W))
    val sigBits = Input(UInt(log2Up(sigWidth).W))
    val kill = Input(Bool())
    /*--------------------------------------------------------------------
    *--------------------------------------------------------------------*/
    val rawOutValid_div = Output(Bool())
    val rawOutValid_sqrt = Output(Bool())
    val roundingModeOut = Output(UInt(3.W))
    val invalidExc = Output(Bool())
    val infiniteExc = Output(Bool())
    val rawOut = Output(new RawFloat(expWidth, sigWidth + 2))
  })

  val sqrtOp_Z       = Reg(Bool())
  val majorExc_Z     = Reg(Bool())
  //*** REDUCE 3 BITS TO 2-BIT CODE:
  val isNaN_Z        = Reg(Bool())
  val isInf_Z        = Reg(Bool())
  val isZero_Z       = Reg(Bool())
  val sign_Z         = Reg(Bool())
  val sExp_Z         = Reg(SInt((expWidth + 2).W))
  val fractB_Z       = Reg(UInt(sigWidth.W))
  val roundingMode_Z = Reg(UInt(3.W))

  /*------------------------------------------------------------------------
  *------------------------------------------------------------------------*/
  val rawA_S = io.a
  val rawB_S = io.b

  //*** IMPROVE THESE:
  val notSigNaNIn_invalidExc_S_div =
    (rawA_S.isZero && rawB_S.isZero) || (rawA_S.isInf && rawB_S.isInf)
  val notSigNaNIn_invalidExc_S_sqrt =
    ! rawA_S.isNaN && ! rawA_S.isZero && rawA_S.sign
  val majorExc_S =
    Mux(io.sqrtOp,
      isSigNaNRawFloat(rawA_S) || notSigNaNIn_invalidExc_S_sqrt,
      isSigNaNRawFloat(rawA_S) || isSigNaNRawFloat(rawB_S) ||
        notSigNaNIn_invalidExc_S_div ||
        (! rawA_S.isNaN && ! rawA_S.isInf && rawB_S.isZero)
    )
  val isNaN_S =
    Mux(io.sqrtOp,
      rawA_S.isNaN || notSigNaNIn_invalidExc_S_sqrt,
      rawA_S.isNaN || rawB_S.isNaN || notSigNaNIn_invalidExc_S_div
    )
  val isInf_S  = Mux(io.sqrtOp, rawA_S.isInf,  rawA_S.isInf || rawB_S.isZero)
  val isZero_S = Mux(io.sqrtOp, rawA_S.isZero, rawA_S.isZero || rawB_S.isInf)
  val sign_S = rawA_S.sign ^ (! io.sqrtOp && rawB_S.sign)

  val specialCaseA_S = rawA_S.isNaN || rawA_S.isInf || rawA_S.isZero
  val specialCaseB_S = rawB_S.isNaN || rawB_S.isInf || rawB_S.isZero
  val normalCase_S_div = ! specialCaseA_S && ! specialCaseB_S
  val normalCase_S_sqrt = ! specialCaseA_S && ! rawA_S.sign
  val normalCase_S = Mux(io.sqrtOp, normalCase_S_sqrt, normalCase_S_div)

  val sExpQuot_S_div =
    rawA_S.sExp +&
      Cat(rawB_S.sExp(expWidth), ~rawB_S.sExp(expWidth - 1, 0)).asSInt
  //*** IS THIS OPTIMAL?:
  val sSatExpQuot_S_div =
    Cat(Mux(((BigInt(7)<<(expWidth - 2)).S <= sExpQuot_S_div),
      6.U,
      sExpQuot_S_div(expWidth + 1, expWidth - 2)
    ),
      sExpQuot_S_div(expWidth - 3, 0)
    ).asSInt

  val evenSqrt_S = io.sqrtOp && ! rawA_S.sExp(0)
  val oddSqrt_S  = io.sqrtOp &&   rawA_S.sExp(0)

  /*------------------------------------------------------------------------
  *------------------------------------------------------------------------*/
  val entering = io.inReady && io.inValid
  val entering_normalCase = entering && normalCase_S

  def extraBits = 4
  val sigA_ext = Cat(rawA_S.sig(sigWidth - 1, 0), 0.U(extraBits.W))

  val sigDs = Module(new SigDivSqrt_srt4(sigWidth + extraBits))

  sigDs.io.in.valid := entering_normalCase
  sigDs.io.in.bits.dsCycles := (io.sigBits + extraBits.U) >> 1
  sigDs.io.kill := io.kill
  sigDs.io.in.bits.sigA := Mux(oddSqrt_S || !io.sqrtOp, sigA_ext, sigA_ext >> 1)

  sigDs.io.in.bits.sigB := Cat(rawB_S.sig(sigWidth - 1, 0), 0.U(extraBits.W))
  sigDs.io.in.bits.isDiv := !io.sqrtOp
  sigDs.io.out.ready := true.B


  val s_idle :: s_comp :: s_special :: Nil = Enum(3)
  val state = RegInit(s_idle)

  val rawOutValid = sigDs.io.out.fire || state === s_special

  switch(state){
    is(s_idle){
      when(entering){
        when(entering_normalCase){
          state := s_comp
        }.otherwise({
          state := s_special
        })
      }
    }
    is(s_comp){
      when(sigDs.io.out.fire){
        state := s_idle
      }
    }
    is(s_special){
      state := s_idle
    }
  }
  when(io.kill){ state := s_idle }

  when(entering){
    sqrtOp_Z   := io.sqrtOp
    majorExc_Z := majorExc_S
    isNaN_Z    := isNaN_S
    isInf_Z    := isInf_S
    isZero_Z   := isZero_S
    sign_Z     := sign_S
    sExp_Z :=
      Mux(io.sqrtOp,
        (rawA_S.sExp>>1) +& (BigInt(1)<<(expWidth - 1)).S,
        sSatExpQuot_S_div
      )
    roundingMode_Z := io.roundingMode
  }

  io.inReady := (state === s_idle) && sigDs.io.in.ready

  /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
  io.rawOutValid_div  := rawOutValid && ! sqrtOp_Z
  io.rawOutValid_sqrt := rawOutValid &&   sqrtOp_Z
  io.roundingModeOut  := roundingMode_Z
  io.invalidExc    := majorExc_Z &&   isNaN_Z
  io.infiniteExc   := majorExc_Z && ! isNaN_Z
  io.rawOut.isNaN  := isNaN_Z
  io.rawOut.isInf  := isInf_Z
  io.rawOut.isZero := isZero_Z
  io.rawOut.sign   := sign_Z
  io.rawOut.sExp   := sExp_Z

  io.rawOut.sig := sigDs.io.out.bits.quotient | !sigDs.io.out.bits.isZeroRem
}

class
DivSqrtRecFNToRaw_srt4(expWidth: Int, sigWidth: Int)
  extends Module
{
  val io = IO(new Bundle {
    /*--------------------------------------------------------------------
    *--------------------------------------------------------------------*/
    val inReady        = Output(Bool())
    val inValid        = Input(Bool())
    val sqrtOp         = Input(Bool())
    val sigBits        = Input(UInt(log2Up(sigWidth).W))
    val kill           = Input(Bool())
    val a              = Input(UInt((expWidth + sigWidth + 1).W))
    val b              = Input(UInt((expWidth + sigWidth + 1).W))
    val roundingMode   = Input(UInt(3.W))
    /*--------------------------------------------------------------------
    *--------------------------------------------------------------------*/
    val rawOutValid_div  = Output(Bool())
    val rawOutValid_sqrt = Output(Bool())
    val roundingModeOut  = Output(UInt(3.W))
    val invalidExc       = Output(Bool())
    val infiniteExc      = Output(Bool())
    val rawOut = Output(new RawFloat(expWidth, sigWidth + 2))
  })

  val divSqrtRawFN =
    Module(new DivSqrtRawFN_srt4(expWidth, sigWidth))

  io.inReady := divSqrtRawFN.io.inReady
  divSqrtRawFN.io.inValid      := io.inValid
  divSqrtRawFN.io.sqrtOp       := io.sqrtOp
  divSqrtRawFN.io.sigBits := io.sigBits
  divSqrtRawFN.io.kill := io.kill
  divSqrtRawFN.io.a            := rawFloatFromRecFN(expWidth, sigWidth, io.a)
  divSqrtRawFN.io.b            := rawFloatFromRecFN(expWidth, sigWidth, io.b)
  divSqrtRawFN.io.roundingMode := io.roundingMode

  io.rawOutValid_div  := divSqrtRawFN.io.rawOutValid_div
  io.rawOutValid_sqrt := divSqrtRawFN.io.rawOutValid_sqrt
  io.roundingModeOut  := divSqrtRawFN.io.roundingModeOut
  io.invalidExc       := divSqrtRawFN.io.invalidExc
  io.infiniteExc      := divSqrtRawFN.io.infiniteExc
  io.rawOut           := divSqrtRawFN.io.rawOut

}

/*----------------------------------------------------------------------------
*----------------------------------------------------------------------------*/

class
DivSqrtRecFN_srt4(expWidth: Int, sigWidth: Int)
  extends Module
{
  val io = IO(new Bundle {
    /*--------------------------------------------------------------------
    *--------------------------------------------------------------------*/
    val inReady        = Output(Bool())
    val inValid        = Input(Bool())
    val sqrtOp         = Input(Bool())
    val a              = Input(UInt((expWidth + sigWidth + 1).W))
    val b              = Input(UInt((expWidth + sigWidth + 1).W))
    val roundingMode   = Input(UInt(3.W))
    val detectTininess = Input(UInt(1.W))
    /*--------------------------------------------------------------------
    *--------------------------------------------------------------------*/
    val outValid_div   = Output(Bool())
    val outValid_sqrt  = Output(Bool())
    val out            = Output(UInt((expWidth + sigWidth + 1).W))
    val exceptionFlags = Output(UInt(5.W))
  })

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val divSqrtRecFNToRaw =
  Module(new DivSqrtRecFNToRaw_srt4(expWidth, sigWidth))

  io.inReady := divSqrtRecFNToRaw.io.inReady
  divSqrtRecFNToRaw.io.inValid      := io.inValid
  divSqrtRecFNToRaw.io.sqrtOp       := io.sqrtOp
  divSqrtRecFNToRaw.io.sigBits := sigWidth.U
  divSqrtRecFNToRaw.io.kill := false.B
  divSqrtRecFNToRaw.io.a            := io.a
  divSqrtRecFNToRaw.io.b            := io.b
  divSqrtRecFNToRaw.io.roundingMode := io.roundingMode

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  io.outValid_div  := divSqrtRecFNToRaw.io.rawOutValid_div
  io.outValid_sqrt := divSqrtRecFNToRaw.io.rawOutValid_sqrt

  val roundRawFNToRecFN =
    Module(new RoundRawFNToRecFN(expWidth, sigWidth, 0))
  roundRawFNToRecFN.io.invalidExc   := divSqrtRecFNToRaw.io.invalidExc
  roundRawFNToRecFN.io.infiniteExc  := divSqrtRecFNToRaw.io.infiniteExc
  roundRawFNToRecFN.io.in           := divSqrtRecFNToRaw.io.rawOut
  roundRawFNToRecFN.io.roundingMode := divSqrtRecFNToRaw.io.roundingModeOut
  roundRawFNToRecFN.io.detectTininess := io.detectTininess
  io.out            := roundRawFNToRecFN.io.out
  io.exceptionFlags := roundRawFNToRecFN.io.exceptionFlags
}
