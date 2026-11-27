// dry.h -- Dhrystone 2.2a shared header (design.md sec.10 F, t06)
//
// Adapted from the public "C, Version 2.2a" (R. Weicker; shell-ified by
// S. Pemberton) for the fw C framework:
//   - no libc: strcpy/strcmp/memcpy/memset provided in tests/dry_lib.c
//     (real out-of-line calls -- they are inside the measured loop)
//   - malloc replaced by two static Rec_Type objects
//   - time() replaced by CLINT mtime (1 tick/cycle) read around the loop
//   - results go to the dram[] scoreboard; tb/tb_fw.v counts cycles
// The benchmark COMPUTATION is unmodified: same statement mix, same call
// structure. Split into dry1.c / dry2.c exactly like the canonical
// dhry_1.c / dhry_2.c two-pass model (separate compilation, no procedure
// merging -- both objects are compiled independently and linked).
#ifndef DRY_H
#define DRY_H

#include <stdint.h>

#define DRUN_DEFAULT 100        /* calibration override: -DDRY_RUNS=n */
#ifndef DRY_RUNS
#define DRY_RUNS DRUN_DEFAULT
#endif

typedef int     One_Thirty;
typedef int     One_Fifty;
typedef char    Capital_Letter;
typedef int     Boolean;
typedef char    Str_30[31];
typedef int     Arr_1_Dim[50];
typedef int     Arr_2_Dim[50][50];

/* canonical NOENUM path: Enumeration is int, Ident_n are 0..4
   (identical representation; keeps the old-style enum arithmetic in
   Func_2/Proc_6 warning-free under -Werror) */
#define Ident_1 0
#define Ident_2 1
#define Ident_3 2
#define Ident_4 3
#define Ident_5 4
typedef int Enumeration;

typedef struct record {
    struct record *Ptr_Comp;
    Enumeration    Discr;
    union {
        struct { Enumeration Enum_Comp;
                 int   Int_Comp;
                 char  Str_Comp[31]; } var_1;
        struct { Enumeration E_Comp_2;
                 char  Str_2_Comp[31]; } var_2;
        struct { char  Ch_1_Comp;
                 char  Ch_2_Comp; }     var_3;
    } variant;
} Rec_Type, *Rec_Pointer;

#ifndef true
#define true 1
#define false 0
#endif
#define Null 0

#define NUL '\0'

#define structassign(d, s) d = s     /* canonical: struct assignment */

/* Ground rule "no procedure merging": clang -O2 would inline the
   same-TU procedures into main/each other (Proc_5/4/2/1, Func_3...).
   Every Proc_/Func_ definition + the libc calls stay REAL calls. */
#define DNOL __attribute__((noinline))

/* library subset (tests/dry_lib.c, out-of-line) */
char *strcpy(char *d, const char *s);
int   strcmp(const char *a, const char *b);
void *memcpy(void *d, const void *s, unsigned long n);
void *memset(void *d, int c, unsigned long n);

/* global variables: defined in dry1.c, referenced by dry2.c */
extern Rec_Pointer     Ptr_Glob, Next_Ptr_Glob;
extern int             Int_Glob;
extern Boolean         Bool_Glob;
extern char            Ch_1_Glob, Ch_2_Glob;
extern Arr_1_Dim       Arr_1_Glob;
extern Arr_2_Dim       Arr_2_Glob;

void        Proc_1(Rec_Pointer Ptr_Val_Par);
void        Proc_2(One_Fifty *Int_Par_Ref);
void        Proc_3(Rec_Pointer *Ptr_Ref_Par);
void        Proc_4(void);
void        Proc_5(void);
void        Proc_6(Enumeration Enum_Val_Par, Enumeration *Enum_Ref_Par);
void        Proc_7(One_Fifty Int_1_Par_Val, One_Fifty Int_2_Par_Val,
                   One_Fifty *Int_Par_Ref);
void        Proc_8(Arr_1_Dim Arr_1_Par_Ref, Arr_2_Dim Arr_2_Par_Ref,
                   int Int_1_Par_Val, int Int_2_Par_Val);
Enumeration Func_1(Capital_Letter Ch_1_Par_Val, Capital_Letter Ch_2_Par_Val);
Boolean     Func_2(Str_30 Str_1_Par_Ref, Str_30 Str_2_Par_Ref);
Boolean     Func_3(Enumeration Enum_Par_Val);

#endif
