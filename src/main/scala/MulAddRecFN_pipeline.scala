package hardfloat

import chisel3._
import chisel3.util._
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}
import consts._

class Stage1ToStage2IO(expWidth: Int, sigWidth: Int) extends Bundle
{
  val rawA, rawB, rawC = new RawFloat(expWidth, sigWidth)
  val CAlignDist = Output(UInt())
  val reduce4CExtra = Output(Bool())
  val ctrlSigs = Output(new MulAddRecFN_interIo(expWidth, sigWidth))
  val roundingMode = Output(UInt(3.W))
  val detectTininess = Output(Bool())
}

class Stage2ToStage3IO(expWidth: Int, sigWidth: Int) extends Bundle
{
  val csaSum = Output(UInt())
  val csaCarry = Output(UInt())
  val ctrlSigs = Output(new MulAddRecFN_interIo(expWidth, sigWidth))
  val roundingMode = Output(UInt(3.W))
  val detectTininess = Output(Bool())
}

class Stage3ToStage4IO(expWidth: Int, sigWidth: Int) extends Bundle {

  val CDom_absSigSum = Output(UInt())
  val CDom_absSigSumExtra = Output(Bool())
  val notCDom_signSigSum = Output(Bool())
  val notCDom_absSigSum = Output(UInt())
  val notCDom_reduced2AbsSigSum = Output(UInt())

  val notCDom_normDistReduced2 = Output(UInt(log2Up((sigWidth * 2 + 2 + 1) / 2).W))
  val notCDom_reduced4SigExtra = Output(Bool())

  val ctrlSigs = Output(new MulAddRecFN_interIo(expWidth, sigWidth))
  val roundingMode = Output(UInt(3.W))
  val detectTininess = Output(Bool())
}

class Stage4ToStage5IO(expWidth: Int, sigWidth: Int) extends Bundle {
  val ctrlSigs = Output(new MulAddRecFN_interIo(expWidth, sigWidth))
  val rawOut = Output(new RawFloat(expWidth, sigWidth+2))
  val invalidExc = Output(Bool())
  val roundingMode = Output(UInt(3.W))
  val detectTininess = Output(Bool())
}

class MulAddRecFN_pipeline_stage1(expWidth: Int, sigWidth: Int) extends Module {
  val io = IO(new Bundle{
    val in = Flipped(DecoupledIO(new MulAddRecFN_pipelineInput(expWidth, sigWidth)))
    val mulAddA = Output(UInt(sigWidth.W))
    val mulAddB = Output(UInt(sigWidth.W))
    val toStage2 = DecoupledIO(new Stage1ToStage2IO(expWidth, sigWidth))
  })


  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  //*** POSSIBLE TO REDUCE THIS BY 1 OR 2 BITS?  (CURRENTLY 2 BITS BETWEEN
  //***  UNSHIFTED C AND PRODUCT):
  val sigSumWidth = sigWidth * 3 + 3

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val rawA = rawFloatFromRecFN(expWidth, sigWidth, io.in.bits.a)
  val rawB = rawFloatFromRecFN(expWidth, sigWidth, io.in.bits.b)
  val rawC = rawFloatFromRecFN(expWidth, sigWidth, io.in.bits.c)

  val signProd = rawA.sign ^ rawB.sign ^ io.in.bits.op(1)
  //*** REVIEW THE BIAS FOR 'sExpAlignedProd':
  val sExpAlignedProd =
    rawA.sExp +& rawB.sExp + (-(BigInt(1)<<expWidth) + sigWidth + 3).S

  val doSubMags = signProd ^ rawC.sign ^ io.in.bits.op(0)

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val sNatCAlignDist = sExpAlignedProd - rawC.sExp
  val posNatCAlignDist = sNatCAlignDist(expWidth + 1, 0)
  val isMinCAlign = rawA.isZero || rawB.isZero || (sNatCAlignDist < 0.S)
  val CIsDominant =
    ! rawC.isZero && (isMinCAlign || (posNatCAlignDist <= sigWidth.U))
  val CAlignDist =
    Mux(isMinCAlign,
      0.U,
      Mux(posNatCAlignDist < (sigSumWidth - 1).U,
        posNatCAlignDist(log2Up(sigSumWidth) - 1, 0),
        (sigSumWidth - 1).U
      )
    )
//  val mainAlignedSigC =
//    Cat(Mux(doSubMags, ~rawC.sig, rawC.sig),
//      Fill(sigSumWidth - sigWidth + 2, doSubMags)
//    ).asSInt>>CAlignDist
  val reduced4CExtra =
    (orReduceBy4(rawC.sig<<((sigSumWidth - sigWidth - 1) & 3)) &
      lowMask(
        CAlignDist>>2,
        //*** NOT NEEDED?:
        //                 (sigSumWidth + 2)>>2,
        (sigSumWidth - 1)>>2,
        (sigSumWidth - sigWidth - 1)>>2
      )
      ).orR
//  val alignedSigC =
//    Cat(mainAlignedSigC>>3,
//      Mux(doSubMags,
//        mainAlignedSigC(2, 0).andR && ! reduced4CExtra,
//        mainAlignedSigC(2, 0).orR  ||   reduced4CExtra
//      )
//    )


  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  io.in.ready := io.toStage2.ready
  io.toStage2.valid := io.in.valid


  io.mulAddA := rawA.sig
  io.mulAddB := rawB.sig

  io.toStage2.bits.ctrlSigs.isSigNaNAny :=
    isSigNaNRawFloat(rawA) || isSigNaNRawFloat(rawB) ||
      isSigNaNRawFloat(rawC)
  io.toStage2.bits.ctrlSigs.isNaNAOrB := rawA.isNaN || rawB.isNaN
  io.toStage2.bits.ctrlSigs.isInfA    := rawA.isInf
  io.toStage2.bits.ctrlSigs.isZeroA   := rawA.isZero
  io.toStage2.bits.ctrlSigs.isInfB    := rawB.isInf
  io.toStage2.bits.ctrlSigs.isZeroB   := rawB.isZero
  io.toStage2.bits.ctrlSigs.signProd  := signProd
  io.toStage2.bits.ctrlSigs.isNaNC    := rawC.isNaN
  io.toStage2.bits.ctrlSigs.isInfC    := rawC.isInf
  io.toStage2.bits.ctrlSigs.isZeroC   := rawC.isZero
  io.toStage2.bits.ctrlSigs.sExpSum   :=
    Mux(CIsDominant, rawC.sExp, sExpAlignedProd - sigWidth.S)
  io.toStage2.bits.ctrlSigs.doSubMags := doSubMags
  io.toStage2.bits.ctrlSigs.CIsDominant := CIsDominant
  io.toStage2.bits.ctrlSigs.CDom_CAlignDist := 0.U //DontCare
  io.toStage2.bits.ctrlSigs.highAlignedSigC := 0.U //DontCare
  io.toStage2.bits.ctrlSigs.bit0AlignedSigC := 0.U //DontCare

  io.toStage2.bits.rawA := rawA
  io.toStage2.bits.rawB := rawB
  io.toStage2.bits.rawC := rawC
  io.toStage2.bits.CAlignDist := CAlignDist
  io.toStage2.bits.reduce4CExtra := reduced4CExtra
  io.toStage2.bits.roundingMode := io.in.bits.roundingMode
  io.toStage2.bits.detectTininess := io.in.bits.detectTininess
}

class MulAddRecFN_pipeline_stage2(expWidth: Int, sigWidth: Int) extends Module {
  val io = IO(new Bundle {
    val fromStage1 = Flipped(DecoupledIO(new Stage1ToStage2IO(expWidth, sigWidth)))
    val mulSum = Input(UInt((2 * sigWidth + 1).W))
	val mulCarry = Input(UInt((2 * sigWidth + 1).W))
    val toStage3 = DecoupledIO(new Stage2ToStage3IO(expWidth, sigWidth))
  })

  val sigSumWidth = sigWidth * 3 + 3

  val (rawA, rawB, rawC) = (io.fromStage1.bits.rawA, io.fromStage1.bits.rawB, io.fromStage1.bits.rawC)
  val doSubMags = io.fromStage1.bits.ctrlSigs.doSubMags
  val CAlignDist = io.fromStage1.bits.CAlignDist
  val reduced4CExtra = io.fromStage1.bits.reduce4CExtra

  val mainAlignedSigC =
    Cat(Mux(doSubMags, ~rawC.sig, rawC.sig),
      Fill(sigSumWidth - sigWidth + 2, doSubMags)
    ).asSInt>>CAlignDist

  val alignedSigC =
    Cat(mainAlignedSigC>>3,
      Mux(doSubMags,
        mainAlignedSigC(2, 0).andR && ! reduced4CExtra,
        mainAlignedSigC(2, 0).orR  ||   reduced4CExtra
      )
    )

  val mulAddC = alignedSigC(sigWidth * 2, 1)

  val csa = Module(new CSA3_2(2 * sigWidth + 1))
  csa.io.in(0) := io.mulSum
  csa.io.in(1) := io.mulCarry
  csa.io.in(2) := mulAddC

  //val mulAddResult = csa.io.out(0) + Cat(csa.io.out(1), 0.U(1.W))

  io.fromStage1.ready := io.toStage3.ready
  io.toStage3.valid := io.fromStage1.valid

  io.toStage3.bits.csaSum := csa.io.out(0)
  io.toStage3.bits.csaCarry := csa.io.out(1)
  //io.toStage3.bits.mulAddResult := mulAddResult
  io.toStage3.bits.ctrlSigs := io.fromStage1.bits.ctrlSigs
  io.toStage3.bits.ctrlSigs.CDom_CAlignDist := CAlignDist(log2Up(sigWidth + 1) - 1, 0)
  io.toStage3.bits.ctrlSigs.highAlignedSigC := alignedSigC(sigSumWidth - 1, sigWidth * 2 + 1)
  io.toStage3.bits.ctrlSigs.bit0AlignedSigC := alignedSigC(0)
  io.toStage3.bits.roundingMode := io.fromStage1.bits.roundingMode
  io.toStage3.bits.detectTininess := io.fromStage1.bits.detectTininess
}

class MulAddRecFN_pipeline_stage3(expWidth: Int, sigWidth: Int) extends Module {
  val io = IO(new Bundle{
    val fromStage2 = Flipped(DecoupledIO(new Stage2ToStage3IO(expWidth, sigWidth)))
    val toStage4 = DecoupledIO(new Stage3ToStage4IO(expWidth, sigWidth))
  })

  val sigSumWidth = sigWidth * 3 + 3

  val mulAddResult = io.fromStage2.bits.csaSum + Cat(io.fromStage2.bits.csaCarry, 0.U(1.W))
  val highAlignedSigC = io.fromStage2.bits.ctrlSigs.highAlignedSigC
  val bit0AlignedSigC = io.fromStage2.bits.ctrlSigs.bit0AlignedSigC
  val doSubMags = io.fromStage2.bits.ctrlSigs.doSubMags
  val opSignC = io.fromStage2.bits.ctrlSigs.signProd ^ doSubMags
  val CDom_CAlignDist = io.fromStage2.bits.ctrlSigs.CDom_CAlignDist
  val sigSum =
    Cat(
      Mux(mulAddResult(sigWidth * 2),
        highAlignedSigC + 1.U,
        highAlignedSigC
      ),
      mulAddResult(sigWidth * 2 - 1, 0),
      bit0AlignedSigC
    )


  val CDom_absSigSum =
    Mux(doSubMags,
      ~sigSum(sigSumWidth - 1, sigWidth + 1),
      Cat(0.U(1.W),
        //*** IF GAP IS REDUCED TO 1 BIT, MUST REDUCE THIS COMPONENT TO 1 BIT TOO:
        highAlignedSigC(sigWidth + 1, sigWidth),
        sigSum(sigSumWidth - 3, sigWidth + 2)
      )
    )
  val CDom_absSigSumExtra =
    Mux(doSubMags,
      ((~sigSum(sigWidth, 1)).asUInt).orR,
      sigSum(sigWidth + 1, 1).orR
    )


  val notCDom_signSigSum = sigSum(sigWidth * 2 + 3)
  val notCDom_absSigSum =
    Mux(notCDom_signSigSum,
      ~sigSum(sigWidth * 2 + 2, 0),
      sigSum(sigWidth * 2 + 2, 0) + doSubMags
    ).asUInt
  val notCDom_reduced2AbsSigSum = orReduceBy2(notCDom_absSigSum)
  val notCDom_normDistReduced2 = countLeadingZeros(notCDom_reduced2AbsSigSum)


/*
  val notCDom_reduced4SigExtra =
    (orReduceBy2(
      (notCDom_reduced2AbsSigSum(sigWidth>>1, 0)<<((sigWidth>>1) & 1)).asUInt) &
      lowMask((notCDom_normDistReduced2>>1).asUInt, 0, (sigWidth + 2)>>2)
      ).orR
*/



  io.fromStage2.ready := io.toStage4.ready
  io.toStage4.valid := io.fromStage2.valid

  io.toStage4.bits.CDom_absSigSum := CDom_absSigSum
  io.toStage4.bits.CDom_absSigSumExtra := CDom_absSigSumExtra

  io.toStage4.bits.notCDom_signSigSum := notCDom_signSigSum
  io.toStage4.bits.notCDom_absSigSum := notCDom_absSigSum
  io.toStage4.bits.notCDom_reduced2AbsSigSum := notCDom_reduced2AbsSigSum
  io.toStage4.bits.notCDom_normDistReduced2 := notCDom_normDistReduced2
  io.toStage4.bits.notCDom_reduced4SigExtra := 0.U //DontCare


  io.toStage4.bits.ctrlSigs := io.fromStage2.bits.ctrlSigs
  io.toStage4.bits.roundingMode := io.fromStage2.bits.roundingMode
  io.toStage4.bits.detectTininess := io.fromStage2.bits.detectTininess
}

class MulAddRecFN_pipeline_stage4(expWidth: Int, sigWidth: Int) extends Module {
  val io = IO(new Bundle{
    val fromStage3 = Flipped(DecoupledIO(new Stage3ToStage4IO(expWidth, sigWidth)))
    val toStage5 = DecoupledIO(new Stage4ToStage5IO(expWidth, sigWidth))
  })
  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val sigSumWidth = sigWidth * 3 + 3

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val roundingMode_min = (io.fromStage3.bits.roundingMode === round_min)

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------

  val highAlignedSigC = io.fromStage3.bits.ctrlSigs.highAlignedSigC
  val bit0AlignedSigC = io.fromStage3.bits.ctrlSigs.bit0AlignedSigC
  val doSubMags = io.fromStage3.bits.ctrlSigs.doSubMags
  val opSignC = io.fromStage3.bits.ctrlSigs.signProd ^ doSubMags
  val CDom_CAlignDist = io.fromStage3.bits.ctrlSigs.CDom_CAlignDist
  /*
  val sigSum =
    Cat(
      Mux(mulAddResult(sigWidth * 2),
        highAlignedSigC + UInt(1),
        highAlignedSigC
      ),
      mulAddResult(sigWidth * 2 - 1, 0),
      bit0AlignedSigC
    )
  */
  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val CDom_sign = opSignC
  val CDom_sExp = io.fromStage3.bits.ctrlSigs.sExpSum - doSubMags.zext

//  val CDom_absSigSum =
//    Mux(doSubMags,
//      ~sigSum(sigSumWidth - 1, sigWidth + 1),
//      Cat(UInt(0, 1),
//        //*** IF GAP IS REDUCED TO 1 BIT, MUST REDUCE THIS COMPONENT TO 1 BIT TOO:
//        highAlignedSigC(sigWidth + 1, sigWidth),
//        sigSum(sigSumWidth - 3, sigWidth + 2)
//      )
//    )
//  val CDom_absSigSumExtra =
//    Mux(doSubMags,
//      ((~sigSum(sigWidth, 1)).asUInt).orR,
//      sigSum(sigWidth + 1, 1).orR
//    )

  val CDom_absSigSum = io.fromStage3.bits.CDom_absSigSum
  val CDom_absSigSumExtra = io.fromStage3.bits.CDom_absSigSumExtra
  val CDom_mainSig =
    (CDom_absSigSum<<CDom_CAlignDist)(
      sigWidth * 2 + 1, sigWidth - 3)
  val CDom_reduced4SigExtra =
    (orReduceBy4((CDom_absSigSum(sigWidth - 1, 0)<<(~sigWidth & 3)).asUInt) &
      lowMask((CDom_CAlignDist>>2).asUInt, 0, sigWidth>>2)).orR
  val CDom_sig =
    Cat(CDom_mainSig>>3,
      CDom_mainSig(2, 0).orR || CDom_reduced4SigExtra ||
        CDom_absSigSumExtra
    )

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  /*
  val notCDom_signSigSum = sigSum(sigWidth * 2 + 3)
  val notCDom_absSigSum =
    Mux(notCDom_signSigSum,
      ~sigSum(sigWidth * 2 + 2, 0),
      sigSum(sigWidth * 2 + 2, 0) + doSubMags
    ).asUInt
  val notCDom_reduced2AbsSigSum = orReduceBy2(notCDom_absSigSum)
  val notCDom_normDistReduced2 = countLeadingZeros(notCDom_reduced2AbsSigSum)
  */

  val notCDom_absSigSum = io.fromStage3.bits.notCDom_absSigSum
  val notCDom_normDistReduced2 = io.fromStage3.bits.notCDom_normDistReduced2
  val notCDom_reduced2AbsSigSum = io.fromStage3.bits.notCDom_reduced2AbsSigSum

  val notCDom_nearNormDist = (notCDom_normDistReduced2<<1).asUInt
  val notCDom_sExp = io.fromStage3.bits.ctrlSigs.sExpSum - notCDom_nearNormDist.zext
  val notCDom_mainSig =
    (notCDom_absSigSum<<notCDom_nearNormDist)(
      sigWidth * 2 + 3, sigWidth - 1)


  val notCDom_reduced4SigExtra =
    (orReduceBy2(
      (notCDom_reduced2AbsSigSum(sigWidth>>1, 0)<<((sigWidth>>1) & 1)).asUInt) &
      lowMask((notCDom_normDistReduced2>>1).asUInt, 0, (sigWidth + 2)>>2)
      ).orR


  val notCDom_signSigSum  = io.fromStage3.bits.notCDom_signSigSum
  //val notCDom_reduced4SigExtra = io.fromStage3.bits.notCDom_reduced4SigExtra
  val notCDom_sig =
    Cat(notCDom_mainSig>>3,
      notCDom_mainSig(2, 0).orR || notCDom_reduced4SigExtra
    )
  val notCDom_completeCancellation =
    (notCDom_sig(sigWidth + 2, sigWidth + 1) === 0.U)
  val notCDom_sign =
    Mux(notCDom_completeCancellation,
      roundingMode_min,
      io.fromStage3.bits.ctrlSigs.signProd ^ notCDom_signSigSum
    )

  //------------------------------------------------------------------------
  //------------------------------------------------------------------------
  val notNaN_isInfProd = io.fromStage3.bits.ctrlSigs.isInfA || io.fromStage3.bits.ctrlSigs.isInfB
  val notNaN_isInfOut = notNaN_isInfProd || io.fromStage3.bits.ctrlSigs.isInfC
  val notNaN_addZeros =
    (io.fromStage3.bits.ctrlSigs.isZeroA || io.fromStage3.bits.ctrlSigs.isZeroB) &&
      io.fromStage3.bits.ctrlSigs.isZeroC

  io.fromStage3.ready := io.toStage5.ready
  io.toStage5.valid := io.fromStage3.valid

  io.toStage5.bits.ctrlSigs := io.fromStage3.bits.ctrlSigs
  io.toStage5.bits.roundingMode := io.fromStage3.bits.roundingMode
  io.toStage5.bits.detectTininess := io.fromStage3.bits.detectTininess
  io.toStage5.bits.invalidExc :=
    io.fromStage3.bits.ctrlSigs.isSigNaNAny ||
      (io.fromStage3.bits.ctrlSigs.isInfA && io.fromStage3.bits.ctrlSigs.isZeroB) ||
      (io.fromStage3.bits.ctrlSigs.isZeroA && io.fromStage3.bits.ctrlSigs.isInfB) ||
      (! io.fromStage3.bits.ctrlSigs.isNaNAOrB &&
        (io.fromStage3.bits.ctrlSigs.isInfA || io.fromStage3.bits.ctrlSigs.isInfB) &&
        io.fromStage3.bits.ctrlSigs.isInfC &&
        io.fromStage3.bits.ctrlSigs.doSubMags)
  io.toStage5.bits.rawOut.isNaN := io.fromStage3.bits.ctrlSigs.isNaNAOrB || io.fromStage3.bits.ctrlSigs.isNaNC
  io.toStage5.bits.rawOut.isInf := notNaN_isInfOut
  //*** IMPROVE?:
  io.toStage5.bits.rawOut.isZero :=
    notNaN_addZeros ||
      (! io.fromStage3.bits.ctrlSigs.CIsDominant && notCDom_completeCancellation)
  io.toStage5.bits.rawOut.sign :=
    (notNaN_isInfProd && io.fromStage3.bits.ctrlSigs.signProd) ||
      (io.fromStage3.bits.ctrlSigs.isInfC && opSignC) ||
      (notNaN_addZeros && ! roundingMode_min &&
        io.fromStage3.bits.ctrlSigs.signProd && opSignC) ||
      (notNaN_addZeros && roundingMode_min &&
        (io.fromStage3.bits.ctrlSigs.signProd || opSignC)) ||
      (! notNaN_isInfOut && ! notNaN_addZeros &&
        Mux(io.fromStage3.bits.ctrlSigs.CIsDominant, CDom_sign, notCDom_sign))
  io.toStage5.bits.rawOut.sExp := Mux(io.fromStage3.bits.ctrlSigs.CIsDominant, CDom_sExp, notCDom_sExp)
  io.toStage5.bits.rawOut.sig := Mux(io.fromStage3.bits.ctrlSigs.CIsDominant, CDom_sig, notCDom_sig)
}

class MulAddRecFN_pipeline_stage5(expWidth: Int, sigWidth: Int) extends Module {
  val io = IO(new Bundle{
    val fromStage4 = Flipped(DecoupledIO(new Stage4ToStage5IO(expWidth, sigWidth)))
    val out = DecoupledIO(new MulAddRecFN_pipelineOutput(expWidth, sigWidth))
  })
  val roundRawFNToRecFN = Module(new RoundRawFNToRecFN(expWidth, sigWidth, 0))
  roundRawFNToRecFN.io.invalidExc   := io.fromStage4.bits.invalidExc
  roundRawFNToRecFN.io.infiniteExc  := false.B
  roundRawFNToRecFN.io.in           := io.fromStage4.bits.rawOut
  roundRawFNToRecFN.io.roundingMode := io.fromStage4.bits.roundingMode
  roundRawFNToRecFN.io.detectTininess := io.fromStage4.bits.detectTininess

  io.fromStage4.ready := io.out.ready
  io.out.valid := io.fromStage4.valid

  io.out.bits.out            := roundRawFNToRecFN.io.out
  io.out.bits.exceptionFlags := roundRawFNToRecFN.io.exceptionFlags
}

class MulAddRecFN_pipelineInput(expWidth: Int, sigWidth: Int) extends Bundle {
  val op = UInt(2.W)
  val a = UInt((expWidth + sigWidth + 1).W)
  val b = UInt((expWidth + sigWidth + 1).W)
  val c = UInt((expWidth + sigWidth + 1).W)
  val roundingMode = UInt(3.W)
  val detectTininess = UInt(1.W)
}

class MulAddRecFN_pipelineOutput(expWidth: Int, sigWidth: Int) extends Bundle {
  val out = Output(UInt((expWidth + sigWidth + 1).W))
  val exceptionFlags = Output(UInt(5.W))
}

class MulAddRecFN_pipeline(expWidth: Int, sigWidth: Int) extends Module {

  val io = IO(new Bundle {
    val in = Flipped(DecoupledIO(new MulAddRecFN_pipelineInput(expWidth, sigWidth)))
    val out = DecoupledIO(new MulAddRecFN_pipelineOutput(expWidth, sigWidth))
  })

  def pipelineConnect[T <: Data](left: DecoupledIO[T], right: DecoupledIO[T]) = {
    val data = RegEnable(left.bits, left.fire())
    val valid = RegInit(false.B)
    left.ready := !valid || right.ready
    right.valid := valid
    right.bits := data
    when(right.fire()){
      valid := false.B
    }
    when(left.fire()){
      valid := true.B
    }
  }

  val stage1 = Module(new MulAddRecFN_pipeline_stage1(expWidth, sigWidth))
  val stage2 = Module(new MulAddRecFN_pipeline_stage2(expWidth, sigWidth))
  val stage3 = Module(new MulAddRecFN_pipeline_stage3(expWidth, sigWidth))
  val stage4 = Module(new MulAddRecFN_pipeline_stage4(expWidth, sigWidth))
  val stage5 = Module(new MulAddRecFN_pipeline_stage5(expWidth, sigWidth))

  stage1.io.in <> io.in
  pipelineConnect(stage1.io.toStage2, stage2.io.fromStage1)
  pipelineConnect(stage2.io.toStage3, stage3.io.fromStage2)
  pipelineConnect(stage3.io.toStage4, stage4.io.fromStage3)
  pipelineConnect(stage4.io.toStage5, stage5.io.fromStage4)
  io.out <> stage5.io.out

  val multiplier = Module(new ArrayMultiplier(
    sigWidth+1,
    regDepth = 0,
    realArraryMult = true,
    hasReg = true
  ))
  multiplier.io.a := stage1.io.mulAddA
  multiplier.io.b := stage1.io.mulAddB
  multiplier.io.reg_en := stage1.io.toStage2.fire()

  stage2.io.mulSum := multiplier.io.sum
  stage2.io.mulCarry := multiplier.io.carry

}
