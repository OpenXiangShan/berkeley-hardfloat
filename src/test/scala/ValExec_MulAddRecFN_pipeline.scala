/*============================================================================

This Chisel source file is part of a pre-release version of the HardFloat IEEE
Floating-Point Arithmetic Package, by John R. Hauser (with some contributions
from Yunsup Lee and Andrew Waterman, mainly concerning testing).

Copyright 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017 The Regents of the
University of California.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions, and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions, and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the University nor the names of its contributors may
    be used to endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS", AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE
DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

package hardfloat.test

import hardfloat._
import Chisel._

class MulAddRecFN_pipeline_io(expWidth: Int, sigWidth: Int) extends Bundle {

  val a = Bits(width = expWidth + sigWidth)
  val b = Bits(width = expWidth + sigWidth)
  val c = Bits(width = expWidth + sigWidth)
  val roundingMode   = UInt(width = 3)
  val detectTininess = UInt(width = 1)

  val out = Bits(width = expWidth + sigWidth)
  val exceptionFlags = Bits(width = 5)
}

class ValExec_MulAddRecFN_pipeline(expWidth: Int, sigWidth: Int) extends Module
{
  val io = new Bundle {
    val input = Flipped(DecoupledIO(new MulAddRecFN_pipeline_io(expWidth, sigWidth)))

    val expected = new Bundle {
      val out = Bits(OUTPUT, expWidth + sigWidth)
      val exceptionFlags = Bits(OUTPUT, 5)
      val recOut = Bits(OUTPUT, expWidth + sigWidth + 1)
    }

    val actual = new Bundle {
      val out = Bits(OUTPUT, expWidth + sigWidth + 1)
      val exceptionFlags = Bits(OUTPUT, 5)
    }

    val check = Bool(OUTPUT)
    val pass = Bool(OUTPUT)
  }

  val mulAddRecFN = Module(new MulAddRecFN_pipeline(expWidth, sigWidth))
  val cq = Module(new Queue(new MulAddRecFN_pipeline_io(expWidth, sigWidth), 5))


  cq.io.enq.valid := io.input.valid && mulAddRecFN.io.in.ready
  cq.io.enq.bits := io.input.bits

  mulAddRecFN.io.in.valid := io.input.valid && cq.io.enq.ready
  mulAddRecFN.io.out.ready := cq.io.deq.valid

  io.input.ready := cq.io.enq.ready && mulAddRecFN.io.in.ready

  mulAddRecFN.io.in.bits.op := UInt(0)
  mulAddRecFN.io.in.bits.a := recFNFromFN(expWidth, sigWidth, io.input.bits.a)
  mulAddRecFN.io.in.bits.b := recFNFromFN(expWidth, sigWidth, io.input.bits.b)
  mulAddRecFN.io.in.bits.c := recFNFromFN(expWidth, sigWidth, io.input.bits.c)
  mulAddRecFN.io.in.bits.roundingMode   := io.input.bits.roundingMode
  mulAddRecFN.io.in.bits.detectTininess := io.input.bits.detectTininess

  io.expected.out := cq.io.deq.bits.out
  io.expected.exceptionFlags := cq.io.deq.bits.exceptionFlags
  io.expected.recOut := recFNFromFN(expWidth, sigWidth, io.expected.out)

  io.actual.out := mulAddRecFN.io.out.bits.out
  io.actual.exceptionFlags := mulAddRecFN.io.out.bits.exceptionFlags

  cq.io.deq.ready := mulAddRecFN.io.out.valid

  io.check := cq.io.deq.fire()
  io.pass := cq.io.deq.fire() &&
	equivRecFN(expWidth, sigWidth, io.actual.out, io.expected.recOut) &&
	  (io.actual.exceptionFlags === io.expected.exceptionFlags)
}


class ValExec_MulAddRecFN_pipeline_add(expWidth: Int, sigWidth: Int) extends Module
{
  val io = new Bundle {
    val input = Flipped(DecoupledIO(new MulAddRecFN_pipeline_io(expWidth, sigWidth)))

    val expected = new Bundle {
      val out = Bits(OUTPUT, expWidth + sigWidth)
      val exceptionFlags = Bits(OUTPUT, 5)
      val recOut = Bits(OUTPUT, expWidth + sigWidth + 1)
    }

    val actual = new Bundle {
      val out = Bits(OUTPUT, expWidth + sigWidth + 1)
      val exceptionFlags = Bits(OUTPUT, 5)
    }

    val check = Bool(OUTPUT)
    val pass = Bool(OUTPUT)
  }

  val mulAddRecFN = Module(new MulAddRecFN_pipeline(expWidth, sigWidth))
  val cq = Module(new Queue(new MulAddRecFN_pipeline_io(expWidth, sigWidth), 5))


  cq.io.enq.valid := io.input.valid && mulAddRecFN.io.in.ready
  cq.io.enq.bits := io.input.bits

  mulAddRecFN.io.in.valid := io.input.valid && cq.io.enq.ready
  mulAddRecFN.io.out.ready := cq.io.deq.valid

  io.input.ready := cq.io.enq.ready && mulAddRecFN.io.in.ready

  mulAddRecFN.io.in.bits.op := UInt(0)
  mulAddRecFN.io.in.bits.a := recFNFromFN(expWidth, sigWidth, io.input.bits.a)
  mulAddRecFN.io.in.bits.b := UInt(BigInt(1)<<(expWidth + sigWidth - 1))
  mulAddRecFN.io.in.bits.c := recFNFromFN(expWidth, sigWidth, io.input.bits.b)
  mulAddRecFN.io.in.bits.roundingMode   := io.input.bits.roundingMode
  mulAddRecFN.io.in.bits.detectTininess := io.input.bits.detectTininess

  io.expected.out := cq.io.deq.bits.out
  io.expected.exceptionFlags := cq.io.deq.bits.exceptionFlags
  io.expected.recOut := recFNFromFN(expWidth, sigWidth, io.expected.out)

  io.actual.out := mulAddRecFN.io.out.bits.out
  io.actual.exceptionFlags := mulAddRecFN.io.out.bits.exceptionFlags

  cq.io.deq.ready := mulAddRecFN.io.out.valid

  io.check := cq.io.deq.fire()
  io.pass := cq.io.deq.fire() &&
	equivRecFN(expWidth, sigWidth, io.actual.out, io.expected.recOut) &&
	  (io.actual.exceptionFlags === io.expected.exceptionFlags)
}

class ValExec_MulAddRecFN_pipeline_mul(expWidth: Int, sigWidth: Int) extends Module
{
  val io = new Bundle {
    val input = Flipped(DecoupledIO(new MulAddRecFN_pipeline_io(expWidth, sigWidth)))

    val expected = new Bundle {
      val out = Bits(OUTPUT, expWidth + sigWidth)
      val exceptionFlags = Bits(OUTPUT, 5)
      val recOut = Bits(OUTPUT, expWidth + sigWidth + 1)
    }

    val actual = new Bundle {
      val out = Bits(OUTPUT, expWidth + sigWidth + 1)
      val exceptionFlags = Bits(OUTPUT, 5)
    }

    val check = Bool(OUTPUT)
    val pass = Bool(OUTPUT)
  }

  val mulAddRecFN = Module(new MulAddRecFN_pipeline(expWidth, sigWidth))
  val cq = Module(new Queue(new MulAddRecFN_pipeline_io(expWidth, sigWidth), 5))


  cq.io.enq.valid := io.input.valid && mulAddRecFN.io.in.ready
  cq.io.enq.bits := io.input.bits

  mulAddRecFN.io.in.valid := io.input.valid && cq.io.enq.ready
  mulAddRecFN.io.out.ready := cq.io.deq.valid

  io.input.ready := cq.io.enq.ready && mulAddRecFN.io.in.ready

  mulAddRecFN.io.in.bits.op := UInt(0)
  mulAddRecFN.io.in.bits.a := recFNFromFN(expWidth, sigWidth, io.input.bits.a)
  mulAddRecFN.io.in.bits.b := recFNFromFN(expWidth, sigWidth, io.input.bits.b)
  mulAddRecFN.io.in.bits.c :=
    ((io.input.bits.a ^ io.input.bits.b) & UInt(BigInt(1)<<(expWidth + sigWidth - 1)))<<1
  mulAddRecFN.io.in.bits.roundingMode   := io.input.bits.roundingMode
  mulAddRecFN.io.in.bits.detectTininess := io.input.bits.detectTininess

  io.expected.out := cq.io.deq.bits.out
  io.expected.exceptionFlags := cq.io.deq.bits.exceptionFlags
  io.expected.recOut := recFNFromFN(expWidth, sigWidth, io.expected.out)

  io.actual.out := mulAddRecFN.io.out.bits.out
  io.actual.exceptionFlags := mulAddRecFN.io.out.bits.exceptionFlags

  cq.io.deq.ready := mulAddRecFN.io.out.valid

  io.check := cq.io.deq.fire()
  io.pass := cq.io.deq.fire() &&
	equivRecFN(expWidth, sigWidth, io.actual.out, io.expected.recOut) &&
	  (io.actual.exceptionFlags === io.expected.exceptionFlags)
}

class MulAddRecFN_pipelineSpec extends FMATester {
  def test(f: Int, fn: String): Seq[String] = {
    test(
      s"MulAddRecF${f}_pipeline${fn match {
        case "add" => "_add"
        case "mul" => "_mul"
        case "mulAdd" => ""
      }}",
      () => fn match {
        case "add" => new ValExec_MulAddRecFN_pipeline_add(exp(f), sig(f))
        case "mul" => new ValExec_MulAddRecFN_pipeline_mul(exp(f), sig(f))
        case "mulAdd" => new ValExec_MulAddRecFN_pipeline(exp(f), sig(f))
      },
      Seq(s"f${f}_${fn}")
    )
  }

  "MulAddRecF16_pipeline" should "pass" in {
    check(test(16, "mulAdd"))
  }
  "MulAddRecF32_pipeline" should "pass" in {
    check(test(32, "mulAdd"))
  }
  "MulAddRecF64_pipeline" should "pass" in {
    check(test(64, "mulAdd"))
  }
  "MulAddRecF16_pipeline_add" should "pass" in {
    check(test(16, "add"))
  }
  "MulAddRecF32_pipeline_add" should "pass" in {
    check(test(32, "add"))
  }
  "MulAddRecF64_pipeline_add" should "pass" in {
    check(test(64, "add"))
  }
  "MulAddRecF16_pipeline_mul" should "pass" in {
    check(test(16, "mul"))
  }
  "MulAddRecF32_pipeline_mul" should "pass" in {
    check(test(32, "mul"))
  }
  "MulAddRecF64_pipeline_mul" should "pass" in {
    check(test(64, "mul"))
  }
}
