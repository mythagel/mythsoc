// dry1.c -- Dhrystone 2.2a pass 1 (main, Proc_1..Proc_5, globals).
// Benchmark statements UNMODIFIED from the canonical dhry_1.c (see dry.h
// header for the adaptation notes). Compiled separately from dry2.c
// (ground rule: separate compilation, no procedure merging).
#include "lib.h"
#include "dry.h"

/* ---- fw framework globals (tests/fw_entry.S + fw_traps.S) ---- */
__attribute__((section(".data.dram"), aligned(16)))
volatile uint32_t dram[1024];
uint32_t _stk_main[1024] __attribute__((aligned(16)));
uint32_t _stk_m[1024]    __attribute__((aligned(16)));
uint32_t _stk_s[1024]    __attribute__((aligned(16)));
uint32_t _sav_m[16]      __attribute__((aligned(16)));
uint32_t _sav_s[16]      __attribute__((aligned(16)));

/* unexpected trap = benchmark invalid: record cause, fail (tb protocol
   0x8000_1000+c). mtvec/stvec point at the shims. */
void m_hnd_c(void)
{
    dram[0] = 0x80001000u | (csr_read(0x342u) & 0xFFu);
    dram[1] = 0x5A5A0000u;
    for (;;) ;
}
void s_hnd_c(void) { m_hnd_c(); }

/* ---- Dhrystone global variables ---- */
Rec_Pointer     Ptr_Glob,
                Next_Ptr_Glob;
int             Int_Glob;
Boolean         Bool_Glob;
char            Ch_1_Glob,
                Ch_2_Glob;
int             Arr_1_Glob [50];
int             Arr_2_Glob [50] [50];

/* malloc replacement (static records; addresses are implementation-
   dependent by design -- the benchmark never compares them to constants) */
static Rec_Type Rec_1, Rec_2;

#define CLINT_MTIME_LO 0x0200BFF8u
#define CLINT_MTIME_HI 0x0200BFFCu

/* 64-bit mtime snapshot (hi,lo,hi retry: guards the lo wrap; on this
   benchmark a wrap (4.29G cycles) cannot occur, but do it right) */
static uint64_t mtime_now(void)
{
    uint32_t hi, lo, hi2;
    do { hi = rd32(CLINT_MTIME_HI); lo = rd32(CLINT_MTIME_LO); hi2 = rd32(CLINT_MTIME_HI); }
    while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

/* scoreboard map (words 16+ are test data; TB +DUMP prints 0..115):
   dram[40] = measured loop cycles (lo), [41] = 0, [42] = DRY_RUNS,
   dram[43] = cycles x1000 / runs (fixed-point avg), [44] = setup cycles */
int main(void)
{
    One_Fifty       Int_1_Loc;
    One_Fifty       Int_2_Loc;
    One_Fifty       Int_3_Loc;
    char            Ch_Index;
    Enumeration     Enum_Loc;
    Str_30          Str_1_Loc;
    Str_30          Str_2_Loc;
    int             Run_Index;
    const int       Number_Of_Runs = DRY_RUNS;
    uint64_t        t0, t1;
    uint32_t        i, j;

    dram[16] = 0;                       /* trap count (must stay 0) */

    /* wb_ram is X until written (rtl/wb_ram.v): the canonical program
       RMW-reads Arr_2 before writing it (Proc_8 `+= 1`) -- zero both
       arrays first (t05 lesson: never branch on / accumulate X) */
    for (i = 0; i < 50; i++) {
        Arr_1_Glob[i] = 0;
        for (j = 0; j < 50; j++)
            Arr_2_Glob[i][j] = 0;
    }

    csr_write(0x305u, (uint32_t)(uintptr_t)m_shim);   /* mtvec */
    csr_write(0x105u, (uint32_t)(uintptr_t)s_shim);   /* stvec */

    /* Initializations */
    Next_Ptr_Glob = &Rec_1;
    Ptr_Glob = &Rec_2;

    Ptr_Glob->Ptr_Comp                    = Next_Ptr_Glob;
    Ptr_Glob->Discr                       = Ident_1;
    Ptr_Glob->variant.var_1.Enum_Comp     = Ident_3;
    Ptr_Glob->variant.var_1.Int_Comp      = 40;
    strcpy (Ptr_Glob->variant.var_1.Str_Comp,
            "DHRYSTONE PROGRAM, SOME STRING");
    strcpy (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");

    Arr_2_Glob [8][7] = 10;
    /* Was missing in published program. Without this statement,
       Arr_2_Glob [8][7] would have an undefined value. */

    uint32_t setup0 = (uint32_t)mtime_now();

    t0 = mtime_now();

    for (Run_Index = 1; Run_Index <= Number_Of_Runs; ++Run_Index)
    {
      Proc_5();
      Proc_4();
        /* Ch_1_Glob == 'A', Ch_2_Glob == 'B', Bool_Glob == true */
      Int_1_Loc = 2;
      Int_2_Loc = 3;
      strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
      Enum_Loc = Ident_2;
      Bool_Glob = ! Func_2 (Str_1_Loc, Str_2_Loc);
        /* Bool_Glob == 1 */
      while (Int_1_Loc < Int_2_Loc)  /* loop body executed once */
      {
        Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
          /* Int_3_Loc == 7 */
        Proc_7 (Int_1_Loc, Int_2_Loc, &Int_3_Loc);
          /* Int_3_Loc == 7 */
        Int_1_Loc += 1;
      } /* while */
        /* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
      Proc_8 (Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
        /* Int_Glob == 5 */
      Proc_1 (Ptr_Glob);
      for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index)
                             /* loop body executed twice */
      {
        if (Enum_Loc == Func_1 (Ch_Index, 'C'))
            /* then, not executed */
          {
          Proc_6 (Ident_1, &Enum_Loc);
          strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
          Int_2_Loc = Run_Index;
          Int_Glob = Run_Index;
          }
      }
        /* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
      Int_2_Loc = Int_2_Loc * Int_1_Loc;
      Int_1_Loc = Int_2_Loc / Int_3_Loc;
      Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
        /* Int_1_Loc == 1, Int_2_Loc == 13, Int_3_Loc == 7 */
      Proc_2 (&Int_1_Loc);
        /* Int_1_Loc == 5 */

    } /* loop "for Run_Index" */

    t1 = mtime_now();
    dram[44] = setup0;                /* cycles from reset to loop start */

    /* ================= canonical final-value verification ============= */
    /* Dhrystone self-check, 2.2a: every result is run-count independent
       except Arr_2_Glob[8][7] */
    chk(1, Int_Glob == 5);
    chk(2, Bool_Glob == 1);
    chk(3, Ch_1_Glob == 'A');
    chk(4, Ch_2_Glob == 'B');
    chk(5, Arr_1_Glob[8] == 7);
    chk(6, Arr_2_Glob[8][7] == Number_Of_Runs + 10);
    chk(7, Ptr_Glob->Discr == 0);
    chk(8, Ptr_Glob->variant.var_1.Enum_Comp == 2);
    chk(9, Ptr_Glob->variant.var_1.Int_Comp == 17);
    chk(10, strcmp(Ptr_Glob->variant.var_1.Str_Comp,
                   "DHRYSTONE PROGRAM, SOME STRING") == 0);
    chk(11, Next_Ptr_Glob->Discr == 0);
    chk(12, Next_Ptr_Glob->variant.var_1.Enum_Comp == 1);
    chk(13, Next_Ptr_Glob->variant.var_1.Int_Comp == 18);
    chk(14, strcmp(Next_Ptr_Glob->variant.var_1.Str_Comp,
                   "DHRYSTONE PROGRAM, SOME STRING") == 0);
    chk(15, Int_1_Loc == 5);
    chk(16, Int_2_Loc == 13);
    chk(17, Int_3_Loc == 7);
    chk(18, Enum_Loc == 1);
    chk(19, strcmp(Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING") == 0);
    chk(20, strcmp(Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING") == 0);

    /* timing sanity: loop must have taken real, plausible cycles */
    chk(21, t1 > t0);
    chk(22, (t1 - t0) < 0x40000000ull);

    dram[40] = (uint32_t)(t1 - t0);
    dram[41] = (uint32_t)((t1 - t0) >> 32);
    dram[42] = (uint32_t)Number_Of_Runs;
    dram[43] = (uint32_t)(((t1 - t0) * 1000ull) / (uint64_t)Number_Of_Runs);
    chk(23, dram[41] == 0);             /* < 2^32 cyc (tool reads [40]) */
    chk(24, dram[16] == 0);             /* no trap happened */

    fw_done();
    return 0;                           /* not reached */
}


DNOL void Proc_1 (Rec_Pointer Ptr_Val_Par)
/******************/

    /* executed once */
{
  Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;
                                        /* == Ptr_Glob_Next */
  /* Local variable, initialized with Ptr_Val_Par->Ptr_Comp,    */
  /* corresponds to "rename" in Ada, "with" in Pascal           */

  structassign (*Ptr_Val_Par->Ptr_Comp, *Ptr_Glob);
  Ptr_Val_Par->variant.var_1.Int_Comp = 5;
  Next_Record->variant.var_1.Int_Comp
        = Ptr_Val_Par->variant.var_1.Int_Comp;
  Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
  Proc_3 (&Next_Record->Ptr_Comp);
    /* Ptr_Val_Par->Ptr_Comp->Ptr_Comp
                        == Ptr_Glob->Ptr_Comp */
  if (Next_Record->Discr == Ident_1)
    /* then, executed */
  {
    Next_Record->variant.var_1.Int_Comp = 6;
    Proc_6 (Ptr_Val_Par->variant.var_1.Enum_Comp,
           &Next_Record->variant.var_1.Enum_Comp);
    Next_Record->Ptr_Comp = Ptr_Glob->Ptr_Comp;
    Proc_7 (Next_Record->variant.var_1.Int_Comp, 10,
           &Next_Record->variant.var_1.Int_Comp);
  }
  else /* not executed */
    structassign (*Ptr_Val_Par, *Ptr_Val_Par->Ptr_Comp);
} /* Proc_1 */


DNOL void Proc_2 (One_Fifty *Int_Par_Ref)
/******************/
    /* executed once */
    /* *Int_Par_Ref == 1, becomes 4 */
{
  One_Fifty  Int_Loc;
  Enumeration   Enum_Loc;

  Int_Loc = *Int_Par_Ref + 10;
  do /* executed once */
    if (Ch_1_Glob == 'A')
      /* then, executed */
    {
      Int_Loc -= 1;
      *Int_Par_Ref = Int_Loc - Int_Glob;
      Enum_Loc = Ident_1;
    } /* if */
  while (Enum_Loc != Ident_1); /* true */
} /* Proc_2 */


DNOL void Proc_3 (Rec_Pointer *Ptr_Ref_Par)
/******************/
    /* executed once */
    /* Ptr_Ref_Par becomes Ptr_Glob */
{
  if (Ptr_Glob != Null)
    /* then, executed */
    *Ptr_Ref_Par = Ptr_Glob->Ptr_Comp;
  Proc_7 (10, Int_Glob, &Ptr_Glob->variant.var_1.Int_Comp);
} /* Proc_3 */


DNOL void Proc_4 (void) /* without parameters */
/*******/
    /* executed once */
{
  Boolean Bool_Loc;

  Bool_Loc = Ch_1_Glob == 'A';
  Bool_Glob = Bool_Loc | Bool_Glob;
  Ch_2_Glob = 'B';
} /* Proc_4 */


DNOL void Proc_5 (void) /* without parameters */
/*******/
    /* executed once */
{
  Ch_1_Glob = 'A';
  Bool_Glob = false;
} /* Proc_5 */
