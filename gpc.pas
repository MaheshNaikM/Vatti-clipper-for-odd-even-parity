(* working pascal /delphi with aet search for merge...
  chain generation also works...old one is useless...
  Warn:triangualtion code works fast with new list handling  ===========================================================================
  full working 23 rdt august  2016 ..tri code is faster then full!
  with intersection sorting also added ..
  Remove proxy references as New merge logic does NOT use them ...
  Project:   Generic Polygon Clipper
  A gourment recipe for getting the difference, intersection,
  exclusive-or or union of arbitrary polygon contours.

  File:      gpc.pas
  Author(old buggy,slow gpc):    Alan Murta (gpc@cs.man.ac.uk)
  Version:   0.0 +++ with intersection sequencing ,lists with tail pointers...

  Copyright: (C) Advanced Interfaces Group,
  University of Manchester.

  This software may be freely copied, modified, and redistributed
  provided that this copyright notice is preserved on all copies.

  The intellectual property rights of the old buggy algorithms  reside
  with the University of Manchester Advanced Interfaces Group.

  You may not distribute this software, in whole or in part, as
  part of any commercial product without the express consent of
  the author.

  There is no warranty or other guarantee of fitness of this
  software for any purpose. It is provided solely "as is".

  ===========================================================================

  Ported to Delphi by Richard B. Winston (rbwinst@usgs.gov) Dec. 17, 2008.
  Based in part on a previous port by Stefan Schedel.

  Mar. 18, 2009 Correction submitted by César Aguilar (cesar.aguilar@gmx.net)
*)

unit gpc;

interface

{$DEFINE USEGR32}

uses
  Windows
{$IFDEF USEGR32},
  Messages, Classes, Graphics, Controls,
  GR32,
  Vcl.Dialogs

{$ENDIF};

// ===========================================================================
// Constants
// ===========================================================================

const
  Version = 'GPC_VERSION "2.32"';
  GPC_EPSILON: double = 2.2204460492503131E-16; { from float.h }

  // ===========================================================================
  // Public Data Types
  // ===========================================================================

type

  Tgpc_op = { Set operation type }
    (GPC_DIFF, { Difference }
    GPC_INT, { Intersection }
    GPC_XOR, { Exclusive or }
    GPC_UNION { Union }
    );

{$IFDEF USEGR32}

  Tgpc_vertex = record { Polygon vertex structure }
    x: double; { Vertex x component }
    y: double; { vertex y component }
  end;
{$ELSE}

  Tgpc_vertex = GR32.TFloatPoint;
{$ENDIF}
  Pgpc_vertex_array = ^Tgpc_vertex_array; { Helper Type for indexing }
  Tgpc_vertex_array = array [0 .. MaxInt div sizeof(Tgpc_vertex) - 1]
    of Tgpc_vertex;

  Pgpc_vertex_list = ^Tgpc_vertex_list; { Vertex list structure }

  Tgpc_vertex_list = record
    num_vertices: integer; { Number of vertices in list }
    vertex: Pgpc_vertex_array; { Vertex array pointer }
  end;

  PIntegerArray = ^TIntegerArray;
  TIntegerArray = array [0 .. MaxInt div sizeof(integer) - 1] of integer;

  Pgpc_vertex_list_array = ^Tgpc_vertex_list_array; { Helper Type for indexing }
  Tgpc_vertex_list_array = array [0 .. MaxInt div sizeof(Tgpc_vertex_list) - 1]
    of Tgpc_vertex_list;

  Pgpc_polygon = ^Tgpc_polygon;

  Tgpc_polygon = record { Polygon set structure }
    num_contours: integer; { Number of contours in polygon }
    hole: PIntegerArray; { Hole / external contour flags }
    contour: Pgpc_vertex_list_array; { Contour array pointer }
  end;

  Pgpc_tristrip = ^Tgpc_tristrip; { Tristrip set structure }

  Tgpc_tristrip = record
    num_strips: integer; { Number of tristrips }
    strip: Pgpc_vertex_list_array; { Tristrip array pointer }
  end;

  Tgpc_ScanLineNode = record
    y: double;
    SLnext: ^Tgpc_ScanLineNode;
  end;
  // ===========================================================================
  // Public Function Prototypes
  // ===========================================================================

procedure gpc_read_polygon(var f: text; read_hole_flags: integer;
  p: Pgpc_polygon);

procedure gpc_write_polygon(var f: text; write_hole_flags: integer;
  p: Pgpc_polygon);

procedure gpc_add_contour(polygon: Pgpc_polygon; contour: Pgpc_vertex_list;
  hole: integer);

procedure gpc_polygon_clip(set_operation: Tgpc_op;
  subject_polygon: Pgpc_polygon; clip_polygon: Pgpc_polygon;
  result_polygon: Pgpc_polygon);

procedure gpc_tristrip_clip(op: Tgpc_op; subj: Pgpc_polygon; clip: Pgpc_polygon;
  Tresult: Pgpc_tristrip);
procedure gpc_tri_poly(op: Tgpc_op; subj: Pgpc_polygon; clip: Pgpc_polygon;
  presult: Pgpc_polygon;  OLDrun:boolean);
procedure gpc_polygon_to_tristrip(s: Pgpc_polygon; t: Pgpc_tristrip);

procedure gpc_free_polygon(polygon: Pgpc_polygon);

procedure gpc_free_tristrip(tristrip: Pgpc_tristrip);
procedure PArea(Gpoly: Pgpc_polygon; var x: double);
procedure TriStripArea(tristrip: Pgpc_tristrip; var StripArea: double);

implementation

uses
  SysUtils,
  Math;



// ===========================================================================
// Constants
// ===========================================================================

const
  DBL_MAX: double = MaxDouble;

  DBL_DIG = 15;

  FFALSE = 0;
  FTRUE = 1;

  LEFT = 0;
  RIGHT = 1;

  ABOVE = 0;
  BELOW = 1;

  clip = 0;
  subj = 1;

  INVERT_TRISTRIPS = FFALSE;



  // ===========================================================================
  // Private Data Types
  // ===========================================================================

type
  Tvertex_type = ( { Edge intersection classes }
    NUL, { Empty non-intersection }
    EMX, { External maximum }
    ELI, { External left intermediate }
    TED, { Top edge }
    ERI, { External right intermediate }
    RED, { Right edge }
    IMM, { Internal maximum and minimum }
    IMN, { Internal minimum }
    EMN, { External minimum }
    EMM, { External maximum and minimum }
    LED, { Left edge }
    ILI, { Internal left intermediate }
    BED, { Bottom edge }
    IRI, { Internal right intermediate }
    IMX, { Internal maximum }
    FUL { Full non-intersection }
    );

  Th_state = { Horizontal edge states }
    (NH, { No horizontal edge }
    BH, { Bottom horizontal edge }
    TH { Top horizontal edge }
    );

  Tbundle_state = (UNBUNDLED, BUNDLE_HEAD, BUNDLE_TAIL);

  PPvertex_node = ^Pvertex_node;
  Pvertex_node = ^Tvertex_node; { Internal vertex list datatype }

  Tvertex_node = record
    x: double; { X coordinate component }
    y: double; { Y coordinate component }
    next: Pvertex_node; { Pointer to next vertex in list }
  end;

  Pvertex_node_array = ^Tvertex_node_array; { Helper type for indexing }
  Tvertex_node_array = array [0 .. 1] of Pvertex_node;

  PPpolygon_node = ^Ppolygon_node;
  Ppolygon_node = ^Tpolygon_node;

  Tpolygon_node = record
    active: integer;
    hole: integer;
    v: array [0 .. 1] of Pvertex_node;
    lastv: array [0 .. 1] of Pvertex_node; // only for Triangles
    next: Ppolygon_node;
   // proxy: Ppolygon_node; // not used with AEL merge scan
  end;

  PPedge_node = ^Pedge_node;
  Pedge_node = ^Tedge_node;

  Tedge_node = record
    vertex: Tgpc_vertex; { Piggy-backed contour vertex data }
    bot: Tgpc_vertex; { Edge lower (x, y) coordinate }
    top: Tgpc_vertex; { Edge upper (x, y) coordinate }
    xb: double; { Scanbeam bottom x coordinate }
    xt: double; { Scanbeam top x coordinate }
    dx: double; { Change in x for a unit y increase }
    typ: integer; { Clip / subject edge flag }
    bundle: array [0 .. 1, 0 .. 1] of integer; { Bundle edge flags }
    bside: array [0 .. 1] of integer; { Bundle left / right indicators }
    bstate: array [0 .. 1] of Tbundle_state; { Edge bundle state }
    outp: array [0 .. 1] of Ppolygon_node; { Output polygon / tristrip pointer }
    prev: Pedge_node; { Previous edge in the AET }
    next: Pedge_node; { Next edge in the AET }
    pred: Pedge_node; { Edge connected at the lower end }
    succ: Pedge_node; { Edge connected at the upper end }
    selnext: Pedge_node; // for Horizontal handling..with HzFirst,HzLast
    selprev: Pedge_node;
    next_bound: Pedge_node; { Pointer to next bound in LMT }
  end;

  PPedge_node_array = ^Pedge_node_array;
  Pedge_node_array = ^Tedge_node_array;
  Tedge_node_array = array [0 .. MaxInt div sizeof(Tedge_node) - 1]
    of Tedge_node;

  PPlmt_node = ^Plmt_node;
  Plmt_node = ^Tlmt_node;

  // In Mpc same lmt_node will be used but with only a single edge at any node
  // and lmt_node may have nodes with identical y
  // sorting is used after all Y minima edges are collected.. M.N....

  Tlmt_node = record { Local minima table }
    y: double; { Y coordinate at local minimum }
    first_bound: Pedge_node;
    { Pointer to bound list of identical y,old Murta code! }
    Base_edge: Pedge_node;
    next: Plmt_node; { Pointer to next local minimum,old Murta code }
  end;

  PPsb_tree = ^Psb_tree;
  Psb_tree = ^Tsb_tree;

  Tsb_tree = record { Scanbeam tree }
    y: double; { Scanbeam node y value }
    less: Psb_tree; { Pointer to nodes with lower y }
    more: Psb_tree; { Pointer to nodes with higher y }
  end;

  PPit_node = ^Pit_node;
  Pit_node = ^Tit_node; { Intersection table }

  Tit_node = record
    ie: array [0 .. 1] of Pedge_node; { Intersecting edge (bundle) pair }
    bt: array [0 .. 1] of Pedge_node; { Intersecting edge (bundle) Tails }
    point: Tgpc_vertex; { Point of intersection }
    next: Pit_node; { The next intersection table node }
  end;

  PPst_node = ^Pst_node;
  Pst_node = ^Tst_node; { Sorted edge table }

  Tst_node = record
    edge: Pedge_node; { Pointer to AET edge }
    xb: double; { Scanbeam bottom x coordinate }
    xt: double; { Scanbeam top x coordinate }
    dx: double; { Change in x for a unit y increase }
    prev: Pst_node; { Previous edge in sorted list }
  end;

  Pbbox = ^Tbbox;

  Tbbox = record { Contour axis-aligned bounding box }
    xmin: double; { Minimum x coordinate }
    ymin: double; { Minimum y coordinate }
    xmax: double; { Maximum x coordinate }
    ymax: double; { Maximum y coordinate }
  end;

  PTSLnode = ^TSLnode;

  TSLnode = record
    SLy: double;
    SLnext: PTSLnode;
  end;

  PbboxArray = ^TbboxArray;
  TbboxArray = array [0 .. MaxInt div sizeof(Tbbox) - 1] of Tbbox;

  PDoubleArray = ^TDoubleArray;
  TDoubleArray = array [0 .. MaxInt div sizeof(double) - 1] of double;



  // ===========================================================================
  // C Macros, defined as function for PASCAL
  // ===========================================================================

function EQ(a, b: double): boolean;
begin
  EQ := abs(a - b) <= GPC_EPSILON
end;

function PREV_INDEX(i, n: integer): integer;
begin
  PREV_INDEX := ((i - 1 + n) mod n);
end;

function NEXT_INDEX(i, n: integer): integer;
begin
  NEXT_INDEX := ((i + 1) mod n);
end;

function OPTIMAL(v: Pgpc_vertex_array; i, n: integer): boolean;
begin
  OPTIMAL := (v[PREV_INDEX(i, n)].y <> v[i].y) or
    (v[NEXT_INDEX(i, n)].y <> v[i].y);
end;


// ===========================================================
function FWD_FIRST(v: Pedge_node_array; i, n: integer): boolean;
begin // 2 point class only
  FWD_FIRST := (v[NEXT_INDEX(i, n)].vertex.y > v[i].vertex.y); // R
end;

function FWD_EDGE(v: Pedge_node_array; i, n: integer): boolean;
begin // edited to include horizontals
  FWD_EDGE := (v[i].vertex.y <= v[NEXT_INDEX(i, n)].vertex.y);
end;

function REV_EDG(v: Pedge_node_array; i, n: integer): boolean;
begin
  REV_EDG := (v[NEXT_INDEX(i, n)].vertex.y <= v[i].vertex.y);
end;
// =============================================

// ==========================================================
procedure MALLOC(var p: pointer; b: integer; s: string);
begin
  GetMem(p, b);
  if (p = nil) and (b <> 0) then
    raise Exception.Create(Format('gpc malloc failure: %s', [s]));
end;

procedure add_vertex(var p: Pvertex_node; x, y: double);
begin
  if p = nil then
  begin
    MALLOC(pointer(p), sizeof(Tvertex_node), 'tristrip vertex creation');
    p.x := x;
    p.y := y;
    p.next := nil;
  end
  else
    { Head further down the list }
    add_vertex(p.next, x, y);
end;

procedure Tadd_vertexL(var Tri: Ppolygon_node; x, y: double);
var
  p: Pvertex_node;
begin
  if (Tri.lastv[LEFT] <> nil) then
    if ((Tri.lastv[LEFT].x = x) AND (Tri.lastv[LEFT].y = y)) then
      exit;

  MALLOC(pointer(p), sizeof(Tvertex_node), 'tristrip vertex creation');
  p.x := x;
  p.y := y;
  p.next := nil;
  if (Tri.lastv[LEFT] <> nil) then
    Tri.lastv[LEFT].next := p
  else
    Tri.v[LEFT] := p;
  Tri.lastv[LEFT] := p;
  Inc(Tri.active);
end;

procedure Tadd_vertexR(var Tri: Ppolygon_node; x, y: double);
var
  p: Pvertex_node;
begin
  if (Tri.lastv[RIGHT] <> nil) then
    if ((Tri.lastv[RIGHT].x = x) AND (Tri.lastv[RIGHT].y = y)) then
      exit;
  MALLOC(pointer(p), sizeof(Tvertex_node), 'tristrip vertex creation');
  p.x := x;
  p.y := y;
  p.next := nil;
  if (Tri.lastv[RIGHT] <> nil) then
    Tri.lastv[RIGHT].next := p
  else
    Tri.v[RIGHT] := p;
  Tri.lastv[RIGHT] := p;
  Inc(Tri.active);
end;

procedure vertexX(var e: Pedge_node; p, s: integer; var x, y: double);
begin // NOT USED
  add_vertex(e.outp[p].v[s], x, y);
  Inc(e.outp[p].active);
end;

procedure vertexL(var e: Pedge_node; p: integer; var x, y: double);
begin
  Tadd_vertexL(e.outp[p], x, y);
  // Inc(e.outp[p].active);
end;

procedure vertexR(var e: Pedge_node; p: integer; var x, y: double);
begin
  Tadd_vertexR(e.outp[p], x, y);
  // Inc(e.outp[p].active);
end;

procedure P_EDGE(var d, e: Pedge_node; p: integer; var i, j: double);
begin
  d := e;
  repeat
    d := d.prev
  until d.outp[p] <> nil;
  i := d.bot.x + d.dx * (j - d.bot.y);
end;

procedure N_EDGE(var d, e: Pedge_node; p: integer; var i, j: double);
begin
  d := e;
  repeat
    d := d.next;
  until d.outp[p] <> nil;
  i := d.bot.x + d.dx * (j - d.bot.y);
end;

procedure Free(var p: pointer);
begin
  // temporary suppress
  FreeMem(p);
  // p := nil;
end;

procedure CFree(var p: pointer);
begin
  if p <> nil then
    Free(p);
end;


// =========start of code frags for scan lines==================
Var
  SLSpareCount: integer;
  SLSpareHeap: PTSLnode;
  mScanLine: PTSLnode;
  sbt_Minimum, sbt_Maximum: double;
  // global Y minimax/maxima of operand scan beams
  AELEDGEBelowH: Pedge_node; // split bondry for merge command  AELReplace
  // AEL:pedge_node;           //Active edge list --Global    ==  aet
  AET: Pedge_node;
  out_poly: Ppolygon_node; // output contours linkd list  --Global
  // Vlmtvect:array[0..50000] of Tlmt_node;  //only temporary
  // Slmtvect:array[0..50000]of Tlmt_node;   //to be replaced by Tlists
  // after testing
  Lmt_NextFreeIndex: integer;
  LmtList_NextFreeIndex: integer;
  lm_List: TList;
  HzFirst, HzLast, HzEdge: Pedge_node;
  prev_edge, next_edge, succ_edge: Pedge_node;
  horiz: array [0 .. 1] of Th_state;
  inn, exists, parity: array [0 .. 1] of integer;
  clipop: Tgpc_op;
  c_heap, s_heap: Pedge_node_array;
  tn, tnn, p, q: Ppolygon_node;
  lt, ltn: Pvertex_node;
  rt, rtn: Pvertex_node;
  TriStripList, TriStripListLast: Ppolygon_node; // triangulation code globals
  itListLast: Tit_node;
  itList: TList; // will contain intersections within a single scan beam..
  // sorted before use....
  Yb, Yt: double; // global beam bondry scan lines
  VattiMessages: TextFile; // for diagnostics...

procedure FreeList(OList: TList);
var
  idx: integer;
  ListSize: integer;
begin
  idx := 0;
  if (OList <> nil) then
  begin
    ListSize := OList.Count;
    if (ListSize > 0) then
      for idx := 0 to ListSize - 1 do
      begin
        // release minima objects....
        dispose(OList[idx]);
      end;
    OList.Clear();
  end;
end;

procedure SLNodeGet(var SLnode: PTSLnode);
begin
  if (SLSpareHeap <> nil) then
  begin
    SLSpareCount := SLSpareCount - 1;
    SLnode := SLSpareHeap;
    SLSpareHeap := SLSpareHeap.SLnext;
  end
  else
  begin
    new(SLnode);
    SLSpareCount := 0;
    // MALLOC(SLnode, sizeof(PTSLnode), 'SLNodeGet');
  end;
  SLnode.SLnext := nil; // safety clearance

end;

procedure SLNodePut(SLnode: PTSLnode);
begin
  SLSpareCount := SLSpareCount + 1;
  SLnode.SLnext := SLSpareHeap;
  SLSpareHeap := SLnode;

end;

Procedure UpdateScanLineQ(const Uy: double);
// This routine is a time hog ! as it is invoked for each new vertex...
// several alternatives can be tried ..including
// evaluating scan lines before clipping starts.....M.N.
// partial eval is avoided by pre adding maximum Y to mScanLine
var
  newSline: PTSLnode;
  slF, slPrev: PTSLnode;
begin
  // do NOT insert values < current base of scan beam
  if (mScanLine = nil) then
  begin
    SLNodeGet(mScanLine);
    mScanLine.SLnext := nil;
    mScanLine.SLy := Uy;
  end
  else if (Uy < mScanLine.SLy) then
  begin
    SLNodeGet(newSline);
    newSline.SLy := Uy;
    newSline.SLnext := mScanLine;
    mScanLine := newSline;
  end
  else
  begin
    slF := mScanLine;
    // Uy < DBL_MAX      and DBL_MAX is a sentinal
    while (Uy > slF.SLy) do
    begin
      slPrev := slF;
      slF := slF.SLnext;
    end;
    if (Uy = slF.SLy) then
      exit; // return; //ie ignores duplicates  ---important----
    SLNodeGet(newSline);
    newSline.SLy := Uy;
    newSline.SLnext := slF;
    slPrev.SLnext := newSline;
  end;
end;

procedure PopScanLineQ(var y: double);
var
  // NOT working as function in XE2
  // Y:double;
  sl2: PTSLnode;
begin
  if (mScanLine = nil) then
  begin // fault ,return terminal scan line
    y := DBL_MAX; // maximum MPREAL_MAX;
    exit;
  end;

  y := mScanLine.SLy;

  if (y <> DBL_MAX) then
  begin
    sl2 := mScanLine;
    mScanLine := mScanLine.SLnext; // should never be nil if sentinalled
    SLNodePut(sl2);
  end;
  // result:=Y;
end;

// ------------------------------------------------------------------------------
function PeepScanLineQ: double;
var
  y: double;
begin // return top value ,but leave it on the list
  y := mScanLine.SLy;
  result := y;
end;

procedure FreeScanLineQ();
var
  slTmp: PTSLnode;
begin // Release all nodes to RT
  slTmp := nil;
  mScanLine := SLSpareHeap;
  while (mScanLine <> nil) do
  begin
    slTmp := mScanLine.SLnext;
    // FREE( m_Scanline );
    dispose(mScanLine);
    mScanLine := slTmp;
  end;
  SLSpareHeap := nil;
  SLSpareCount := 0;

end;
// ================end of Scan Line code frags==================================
// ===========================================================================
// Global Data
// ===========================================================================

{ Horizontal edge state transitions within scanbeam boundary }
const
  next_h_state: array [0 .. 2, 0 .. 5] of Th_state =
  { ABOVE     BELOW     CROSS }
  { L   R     L   R     L   R }
  { NH } ((BH, TH, TH, BH, NH, NH),
    { BH } (NH, NH, NH, NH, TH, TH),
    { TH } (NH, NH, NH, NH, BH, BH));



  // ===========================================================================
  // Private Functions
  // ===========================================================================

procedure xreset_it(var it: Pit_node);
var
  itn: Pit_node;
begin
  while (it <> nil) do
  begin
    itn := it.next;
    Free(pointer(it));
    it := itn;
  end;
end;

procedure reset_lmt(var lmt: Plmt_node);
var
  lmtn: Plmt_node;
begin
  while lmt <> nil do
  begin
    lmtn := lmt^.next;
    dispose(pointer(lmt)); // allocated with new
    lmt := lmtn;
  end;
end;

// ============the lmt =====================
procedure insert_bound(b: PPedge_node_array; e: Pedge_node_array);
var
  existing_bound: pointer;
begin
  // store the lm in y,dx order
  // it would be much much faster if the lmt entries are filled and then the
  // lmt is sorted on x,dx..!
  if b^ = nil then
  begin
    { Link node e to the tail of the list }
    b^ := e;
  end
  else
  begin
    { Do primary sort on the x field }
    if ((e[0].bot.x < b^[0].bot.x)) then
    begin
      { Insert a new node mid-list }
      existing_bound := b^;
      b^ := e;
      b^[0].next_bound := existing_bound;
    end
    else
    begin
      if ((e[0].bot.x = b^[0].bot.x)) then
      begin
        { Do secondary sort on the dx field }
        if ((e[0].dx < b^[0].dx)) then
        begin
          { Insert a new node mid-list }
          existing_bound := b^;
          b^ := e;
          b^[0].next_bound := existing_bound;
        end
        else
        begin
          { Head further down the list }
          insert_bound(@(b^[0].next_bound), e);
        end;
      end
      else
      begin
        { Head further down the list }
        insert_bound(@(b^[0].next_bound), e);
      end;
    end;
  end;
end;

procedure insert_lmtList(y: double; Fe, Re: Pedge_node_array);
var
  lmt: Plmt_node;
begin
  // take from our own pool.......
  lmt := new(Plmt_node);
  // lmt:=@Slmtvect[LmtList_NextFreeIndex];
  Inc(LmtList_NextFreeIndex);
  lmt.y := y;
  lmt.Base_edge := @Re[0];
  lm_List.Add(pointer(lmt));

  lmt := new(Plmt_node);
  // lmt:=@Slmtvect[LmtList_NextFreeIndex];
  Inc(LmtList_NextFreeIndex);
  lmt.y := y;
  lmt.Base_edge := @Fe[0];
  lm_List.Add(pointer(lmt));
end;

function bound_list(var lmt: Plmt_node; y: double): PPedge_node_array;
var
  existing_node: Plmt_node;
begin
  if lmt = nil then
  begin
    { Add node onto the tail end of the LMT }
    // MALLOC(pointer(lmt), sizeof(Tlmt_node), 'LMT insertion');
    // lmt:=@Vlmtvect[Lmt_NextFreeIndex];
    lmt := new(Plmt_node);
    Inc(Lmt_NextFreeIndex);
    lmt.y := y;
    lmt.first_bound := nil;
    lmt.next := nil;
    result := @lmt.first_bound;
  end
  else if (y < lmt.y) then
  begin
    { Insert a new LMT node before the current node }
    existing_node := lmt;
    // MALLOC(pointer(lmt), sizeof(Tlmt_node), 'LMT insertion');
    // lmt:=@Vlmtvect[Lmt_NextFreeIndex];
    lmt := new(Plmt_node);
    Inc(Lmt_NextFreeIndex);
    lmt.y := y;
    lmt.first_bound := nil;
    lmt.next := existing_node;
    result := @lmt.first_bound;
  end
  else if (y > lmt.y) then
    { Head further up the LMT }
    result := bound_list(lmt.next, y)
  else
    { Use this existing LMT node }
    result := @lmt.first_bound;

end;

// ======================old scan beam tree==================
procedure add_to_sbtree(var entries: integer; var sbtree: Psb_tree;
  const y: double);
begin
  if sbtree = nil then
  begin
    { Add a new tree node here }
    MALLOC(pointer(sbtree), sizeof(Tsb_tree), 'scanbeam tree insertion');
    sbtree.y := y;
    sbtree.less := nil;
    sbtree.more := nil;
    Inc(entries);
  end
  else
  begin
    if (sbtree.y > y) then
    begin
      { Head into the 'less' sub-tree }
      add_to_sbtree(entries, sbtree.less, y);
    end
    else
    begin
      if (sbtree.y < y) then
      begin
        { Head into the 'more' sub-tree }
        add_to_sbtree(entries, sbtree.more, y);
      end;
    end;
  end;
end;

procedure build_sbt(var entries: integer; var sbt: PDoubleArray;
  sbtree: Psb_tree);
begin
  if sbtree.less <> nil then
    build_sbt(entries, sbt, sbtree.less);
  sbt[entries] := sbtree.y;
  Inc(entries);
  if sbtree.more <> nil then
    build_sbt(entries, sbt, sbtree.more);
end;

procedure free_sbtree(var sbtree: Psb_tree);
begin
  if sbtree <> nil then
  begin
    free_sbtree(sbtree.less);
    free_sbtree(sbtree.more);
    Free(pointer(sbtree));
  end;
end;
// =====new code to provide scan beam levels==

function count_optimal_vertices(c: Tgpc_vertex_list): integer;
var
  i: integer;
begin
  result := 0;

  { Ignore non-contributing contours }
  if c.num_vertices > 0 then
  begin
    for i := 0 to c.num_vertices - 1 do
      { Ignore superfluous vertices embedded in horizontal edges }
      if OPTIMAL(c.vertex, i, c.num_vertices) then
        Inc(result);
  end;
end;

procedure TraceChain(Tracedirection: boolean; e, edge_table: Pedge_node_array;
  num_vertices: integer; min: integer; num_edges: integer; typ: integer;
  op: Tgpc_op);
var
  i: integer;
  v: integer;
begin
  v := min;
  e[0].bstate[BELOW] := UNBUNDLED;
  e[0].bundle[BELOW][clip] := FFALSE;
  e[0].bundle[BELOW][subj] := FFALSE;

  for i := 0 to num_edges - 1 do
  begin
    e[i].xb := edge_table[v].vertex.x;
    e[i].bot.x := edge_table[v].vertex.x;
    e[i].bot.y := edge_table[v].vertex.y;

    // =====the next 3 statements are added here
    // as this code allows horizontal at chain ends which
    // are skipped over
    // N  e[i].bstate[BELOW] := UNBUNDLED;
    // R  e[i].bundle[BELOW][clip] := FFALSE;
    // Q  e[i].bundle[BELOW][subj] := FFALSE;

    if (Tracedirection) then
      v := NEXT_INDEX(v, num_vertices)
    else
      v := PREV_INDEX(v, num_vertices); // reverse direction

    e[i].top.x := edge_table[v].vertex.x;
    e[i].top.y := edge_table[v].vertex.y;
    if (e[i].top.y = e[i].bot.y) then
      e[i].dx := 0.0
    else
      e[i].dx := (edge_table[v].vertex.x - e[i].bot.x) /
        (e[i].top.y - e[i].bot.y);
    e[i].typ := typ;
    e[i].outp[ABOVE] := nil;
    e[i].outp[BELOW] := nil;
    e[i].next := nil;
    e[i].prev := nil;
    if (num_edges > 1) and (i < (num_edges - 1)) then
      e[i].succ := @e[i + 1]
    else
      e[i].succ := nil;
    if (num_edges > 1) and (i > 0) then
      e[i].pred := @e[i - 1]
    else
      e[i].pred := nil;
    e[i].next_bound := nil;
    if op = GPC_DIFF then
      e[i].bside[clip] := RIGHT
    else
      e[i].bside[clip] := LEFT;
    e[i].bside[subj] := LEFT;
  end;

end;

function build_lmt(var lmt: Plmt_node; p: Pgpc_polygon; typ: integer;
  op: Tgpc_op): Pedge_node_array;

var
  c, i, min, max, num_edges, v, num_vertices: integer;
  total_vertices, e_index: integer;
  Fe, Re, edge_table: Pedge_node_array;
  vtxY: double;
  FwdCnt, REvCnt: integer;
  StartVindex: integer;
begin
  total_vertices := 0;
  e_index := 0;

  for c := 0 to p.num_contours - 1 do
    Inc(total_vertices, count_optimal_vertices(p.contour[c]));

  { Create the entire input polygon edge table in one go }
  MALLOC(pointer(edge_table), (total_vertices) * sizeof(Tedge_node),
    'edge table creation');

  for c := 0 to p.num_contours - 1 do
  begin
    if p.contour[c].num_vertices < 0 then
    begin
      { Ignore the non-contributing contour and repair the vertex count }
      p.contour[c].num_vertices := -p.contour[c].num_vertices;
    end
    else
    begin
      { Perform contour optimisation(Murta IPR) ,middle of a 3 vertex horizontal  is dropped.. }
      FwdCnt := 0;
      REvCnt := 0;
      num_vertices := 0;
      for i := 0 to p.contour[c].num_vertices - 1 do
        if (OPTIMAL(p.contour[c].vertex, i, p.contour[c].num_vertices)) then
        begin
          edge_table[num_vertices].vertex.x := p.contour[c].vertex[i].x;
          edge_table[num_vertices].vertex.y := p.contour[c].vertex[i].y;
          vtxY := p.contour[c].vertex[i].y;
          // the next statement was replaced in the Mumbai coder...
          { Record vertex in the scanbeam table .NOT required in Lazy agony method }
          // get sbt_minimum and sbt_maximum also,as add_to_sbtree is being dropped...
          if (sbt_Minimum > vtxY) then
            sbt_Minimum := vtxY
          else if (sbt_Maximum < vtxY) then
            sbt_Maximum := vtxY;
          // required only in the IPR claimed so slow ,so slow version of Murta 2.32.....
          // add_to_sbtree(sbt_entries, sbtree, edge_table[num_vertices].vertex.y);
          Inc(num_vertices);
        end;
      // decide to include/exclude first chain if partial
      // Imp:First forward chain may have to be ignored if partial.....

      min := 0;
      while (min < num_vertices) do
      // for min := 0 to num_vertices - 1 do
      begin
        { If a forward local minimum... ,if flat  contour exit ? ! }
        if FWD_FIRST(edge_table, min, num_vertices) then
        begin
          { now  Search for the next local maximum... }
          num_edges := 0;
          max := min;
          while (FWD_EDGE(edge_table, max, num_vertices)) do
          begin
            Inc(num_edges);
            max := NEXT_INDEX(max, num_vertices);
          end;
          break;
          // keep this vertex as base vertex from which the split starts
          // rev,fwd,rev,fwd.......
        end;
        Inc(min);
      end;
      StartVindex := max;
      // now continue the traverse of the contour ,splitting it at up/down vertices
      REPEAT

        // a reverse chain
        num_edges := 0;
        while REV_EDG(edge_table, max, num_vertices) do
        begin
          Inc(num_edges);
          max := NEXT_INDEX(max, num_vertices);
        end;
        { Build the previous edge list }
        Re := @edge_table[e_index];
        Inc(e_index, num_edges);
        Inc(REvCnt);
        // if(e_index > total_vertices) then  break;

        TraceChain(false, Re, edge_table, num_vertices, max, num_edges,
          typ, op);

        // insert_lmtList( edge_table[max].vertex.y, Re);

        // forward chain
        min := max;
        { Search for the next local maximum... }
        num_edges := 0;

        while (FWD_EDGE(edge_table, max, num_vertices)) do
        begin
          Inc(num_edges);
          max := NEXT_INDEX(max, num_vertices);
        end;

        { Build the next edge list }
        Fe := @edge_table[e_index];
        Inc(e_index, num_edges);
        if (e_index > total_vertices) then
          break;
        TraceChain(true, Fe, edge_table, num_vertices, min, num_edges, typ, op);
        insert_lmtList(edge_table[min].vertex.y, Re, Fe);
        Inc(FwdCnt);

      UNTIL max = StartVindex;

    end;
  end;

  if (e_index <> total_vertices) then
    result := nil
    // sort the lmt on y of lmt node and x,dx of the edge at the node
    // sort.... unstable ...quick sort from Runtime....
  else
    result := edge_table;
end;

procedure add_edge_to_aet(var AET: Pedge_node; var edge: Pedge_node;
  prev: Pedge_node);
begin
  while (edge.bot.y = edge.top.y) do
  begin
    edge := edge.succ; // skip horizontals
    if (edge = nil) then
      exit;
  end;
  if AET = nil then
  begin
    { Append edge onto the tail end of the AET }
    AET := edge;
    edge.prev := prev;
    edge.next := nil;
  end
  else
  begin
    { Do primary sort on the xb field }
    if (edge.xb < AET.xb) then
    begin
      { Insert edge here (before the AET edge) }
      edge.prev := prev;
      edge.next := AET;
      AET.prev := edge;
      AET := edge;
    end
    else
    begin
      if (edge.xb = AET.xb) then
      begin
        { Do secondary sort on the dx field }
        if (edge.dx < AET.dx) then
        begin
          { Insert edge here (before the AET edge) }
          edge.prev := prev;
          edge.next := AET;
          AET.prev := edge;
          AET := edge;
        end
        else
        begin
          { Head further into the AET }
          add_edge_to_aet(AET.next, edge, AET);
        end;
      end
      else
      begin
        { Head further into the AET }
        add_edge_to_aet(AET.next, edge, AET);
      end;
    end;
  end;
  // specially for a edge staring a chain after a Murta sequence...
  edge.bstate[BELOW] := UNBUNDLED;
  edge.bstate[ABOVE] := UNBUNDLED; // to get Neater listing in LISTAEL
  edge.bundle[BELOW][clip] := FFALSE;
  edge.bundle[BELOW][subj] := FFALSE;
  edge.outp[BELOW] := nil;
  edge.outp[ABOVE] := nil;
end;

// Replace with a list /collection for intersections
// then sort the list in y order just before use....
// IN Delphi Tlist would serve the purpose...
procedure add_intersection(edge0, edge1: Pedge_node; x, y: double);
var
  it: Pit_node;
begin
  // add to it_List ,which will be sorted before use
  begin
    { Append a new node to the tail of the list }
    // MALLOC(pointer(it), sizeof(Tit_node), 'IT insertion');
    it := new(Pit_node);
    it.ie[0] := edge0;
    it.ie[1] := edge1;
    it.point.x := x;
    it.point.y := y;
    it.next := nil;
    itList.Add(pointer(it));
  end

end;

procedure add_st_edge(var st: Pst_node; edge: Pedge_node);
// dy: double);
var
  existing_node: Pst_node;
  den, x, y, r: double;
  dy: double;
begin
  if st = nil then
  begin
    { Append edge onto the tail end of the ST }
    MALLOC(pointer(st), sizeof(Tst_node), 'ST insertion');
    st.edge := edge;
    st.xb := edge.xb;
    st.xt := edge.xt;
    st.dx := edge.dx;
    st.prev := nil;
  end
  else
  begin
    den := (st.xt - st.xb) - (edge.xt - edge.xb);
    dy := Yt - Yb;
    { If new edge and ST edge don't cross }
    if ((edge.xt >= st.xt) or (edge.dx = st.dx) or (abs(den) <= GPC_EPSILON))
    then
    begin
      { No intersection - insert edge here (before the ST edge) }
      existing_node := st;
      MALLOC(pointer(st), sizeof(Tst_node), 'ST insertion');
      st.edge := edge;
      st.xb := edge.xb;
      st.xt := edge.xt;
      st.dx := edge.dx;
      st.prev := existing_node;
    end
    else
    begin
      { Compute intersection between new edge and ST edge }
      r := (edge.xb - st.xb) / den;
      x := st.xb + r * (st.xt - st.xb);
      y := r * (dy);

      { Insert the edge pointers and the intersection point in the IT }
      // add_intersection(it, st.edge, edge, x, y);
      add_intersection(st.edge, edge, x, y);
      { Head further into the ST }
      add_st_edge(st.prev, edge); // , dy);

    end;
  end;
end;

function TailForEdge(var e: Pedge_node): Pedge_node;
Var
  eprev_Aedge, eT: Pedge_node;
begin
  eT := e;
  if (e <> NIL) then
  begin
    if (e.bstate[ABOVE] = BUNDLE_HEAD) then
    begin
      eprev_Aedge := e.prev;
      while (eprev_Aedge <> NIL) AND
        (eprev_Aedge.bstate[ABOVE] = BUNDLE_TAIL) do
      begin
        eT := eprev_Aedge;
        eprev_Aedge := eprev_Aedge.prev;
      end;
    end;
  end;
  TailForEdge := eT;
end;

procedure TailForIntersectionNodeEdges();
var
  ipN: Pit_node;
begin
  if (itList.Count > 0) then
  begin
    ipN := Pit_node(itList[0]);
    while (ipN <> NIL) do
    begin
      ipN.bt[0] := TailForEdge(ipN.ie[0]);
      ipN.bt[1] := TailForEdge(ipN.ie[1]);
      ipN := ipN.next;
    end;
  end;
end;

// ==================Scan Line Q handling code ===============
function intXCompare(intxL, intxR: pointer): integer;
var
  dy, dx: double;
begin
  dy := Pit_node(intxL).point.y - Pit_node(intxR).point.y;
  if dy < 0 then
    intXCompare := -1
  else if dy > 0 then
    intXCompare := 1
  else
  begin
    dx := Pit_node(intxL).point.x - Pit_node(intxR).point.x;
    if dx < 0 then
      intXCompare := -1
    else if dx > 0 then
      intXCompare := 1
    else
      intXCompare := 0;
  end;
end;

procedure NextLinkForItNodes();
// should be called only after sorting ...
// the next ponniter is just a duplication of index in case of Tlist
var
  nodeCount: integer;
  elementidx: integer;
begin
  nodeCount := itList.Count;
  if (nodeCount > 0) then
  begin
    for elementidx := 0 to nodeCount - 2 do
      Pit_node(itList[elementidx]).next := itList[elementidx + 1];
    Pit_node(itList[nodeCount - 1]).next := nil;
  end;
end;

// ==============================================================
procedure build_intersection_table(AET: Pedge_node);
// dy: double);
var
  st, stp: Pst_node;
  edge: Pedge_node;
begin

  { Build intersection table for the current scanbeam }
  itList.Clear();
  st := nil;

  { Process each AET edge }
  edge := AET;
  while edge <> nil do
  begin
    if (edge.bstate[ABOVE] = BUNDLE_HEAD) or (edge.bundle[ABOVE][clip] <> 0) or
      (edge.bundle[ABOVE][subj] <> 0) then
      add_st_edge(st, edge);
    edge := edge.next;
  end;

  { Free the sorted edge table }
  while st <> nil do
  begin
    stp := st.prev;
    Free(pointer(st));
    st := stp;
  end;
  if (itList.Count > 0) then
  begin
    itList.Sort(intXCompare); // quick sort the intersections within beam
    NextLinkForItNodes(); // do this after sorting only...
    TailForIntersectionNodeEdges(); // send first element as para
  end;
end;

function count_contours(polygon: Ppolygon_node): integer;
var
  nv: integer;
  v, nextv: Pvertex_node;
begin

  result := 0;
  while polygon <> nil do
  begin
    if polygon.active <> 0 then
    begin
      { Count the vertices in the current contour }
      nv := 0;
      v := polygon.v[LEFT];
      while v <> nil do
      begin
        Inc(nv);
        v := v.next;
      end;

      { Record valid vertex counts in the active field }
      if (nv > 2) then
      begin
        polygon.active := nv;
        Inc(result);
      end
      else
      begin
        { Invalid contour: just free the heap }
        v := polygon.v[LEFT];
        while v <> nil do
        begin
          nextv := v.next;
          Free(pointer(v));
          v := nextv;
        end;
        polygon.active := 0;
      end;
    end;

    polygon := polygon.next;
  end;
end;

procedure add_left(p: Ppolygon_node; x, y: double);
var
  nv: Pvertex_node;
begin
  if ((p.v[LEFT].x <> x) or (p.v[LEFT].y <> y)) then
  begin { Create a new vertex node and set its fields }
    MALLOC(pointer(nv), sizeof(Tvertex_node), 'vertex node creation');
    nv.x := x;
    nv.y := y;

    { Add vertex nv to the left end of the polygon's vertex list }
    nv.next := p.v[LEFT];

    { Update proxy[LEFT] to point to nv }
    p.v[LEFT] := nv;
  end;
end;

// ======single routine for all merges,left,right,within beam,across scan line ==
procedure AETReplacePbyQ(p, q: Ppolygon_node);

// Secret===use either the only one above
// If Not above use
// the 2nd of the beloW occurance for right...ignoring 1st....if ELI NOT edited
// This expects that the AI algorithm is not failure prone...
// AELBELOWH should  be set to NULL for [ABOVE] search
//Now we use a split search of the AEL edges ...
var
  edgeInAEL, edgeT: Pedge_node;

  P_proxyfound: boolean;
begin
  // current edge pairs  associated with p,q are NOT relevant  in all merges
  edgeInAEL := AET; // AEL is =aet at global level ...edit to spice up later..
  P_proxyfound := false;

  edgeT := AELEDGEBelowH; // in within Beam Intersections edgeT==NULL

  while (edgeInAEL <> edgeT) do
  begin // ABOVE collection
    if (edgeInAEL.outp[ABOVE] <> nil) then
      if (edgeInAEL.outp[ABOVE] = p) then
      begin
        edgeInAEL.outp[ABOVE] := q;
        P_proxyfound := true;
        break; // speed exit  ,as there is at most one occurance !
      end;
    edgeInAEL := edgeInAEL.next;
  end;
  // If No Horizontal   calls then == p must occur in [ABOVE] ..!
  // ===================================================
  // BELOW collection       useful  only for merge along SCanline divider

  if (NOT P_proxyfound) then
  begin
    edgeInAEL := edgeT.next;
    // Use global current position of edge within AEL in scan loop
    while (edgeInAEL <> nil) do
    begin // below collection
      if (edgeInAEL.outp[BELOW] <> nil) then
        if (edgeInAEL.outp[BELOW] = p) then
        begin
          edgeInAEL.outp[BELOW] := q;
          P_proxyfound := true;
          break; // speed exit !
        end;
      edgeInAEL := edgeInAEL.next;
    end;
  end;
  if (NOT P_proxyfound) then
  begin
    p.active := 0; // destruct error ,Report back to users ? how?
  end
  // fprintf(Doutfile,"AELReplacePbyQ [P NOT found while merging] \n");
  else
    p.active := 0; // Important ...
end;

// ================================================
procedure merge_left(p: Ppolygon_node; q: Ppolygon_node);
begin
  { Label contour as a hole }
  q.hole := FTRUE;
 if p = q then     exit;
 { Assign P's vertex list to the left end of Q's list }
  p.v[RIGHT].next := q.v[LEFT];
  q.v[LEFT] := p.v[LEFT];
  { Replace any P references to Q in aet }
  AETReplacePbyQ(p, q);
end;

procedure add_right(p: Ppolygon_node; x, y: double);
var
  nv: Pvertex_node;
begin
  if ((p.v[RIGHT].x <> x) or (p.v[RIGHT].y <> y)) then
  begin
    { Create a new vertex node and set its fields }
    MALLOC(pointer(nv), sizeof(Tvertex_node), 'vertex node creation');
    nv.x := x;
    nv.y := y;
    nv.next := nil;

    { Add vertex nv to the right end of the polygon's vertex list }
    p.v[RIGHT].next := nv;

    { Update proxy.v[RIGHT] to point to nv }
    p.v[RIGHT] := nv;
  end;
end;

procedure merge_right(p: Ppolygon_node; q: Ppolygon_node);
begin
  { Label contour as external }
  q.hole := FFALSE;
 if p = q then   exit;
  { Assign P's vertex list to the right end of Q's list }
  q.v[RIGHT].next := p.v[LEFT];
  q.v[RIGHT] := p.v[RIGHT];
  AETReplacePbyQ(p, q);
 end;

procedure add_local_min(edge: Pedge_node; x, y: double);
var // parameter  p: PPpolygon_node;
  nv: Pvertex_node;
  existing_min: Ppolygon_node;
  p: Ppolygon_node;
begin
  // use global out_poly

  MALLOC(pointer(p), sizeof(Tpolygon_node), 'polygon node creation');

  { Create a new vertex node and set its fields }
  MALLOC(pointer(nv), sizeof(Tvertex_node), 'vertex node creation');
  nv.x := x;
  nv.y := y;
  nv.next := nil;

  { Initialise proxy to point to p itself }
//  p^.proxy := p;       //this attribut has been removed ...
                         //was used  for merging contours
  p^.active := FTRUE;
  p^.next := out_poly;
  p^.hole := FFALSE;
  { Make v[LEFT] and v[RIGHT] point to new vertex nv }
  p^.v[LEFT] := nv;
  p^.v[RIGHT] := nv;

  { Assign polygon p to the edge }
  edge.outp[ABOVE] := p;
  out_poly := p;
end;

function count_tristrips(tn: Ppolygon_node): integer;
begin
  result := 0;

  while tn <> nil do
  begin
    if tn.active > 2 then
      Inc(result);
    tn := tn.next;
  end;
end;

// =============================================================
procedure new_tristrip(edge: Pedge_node; x, y: double);
var
  tn: Ppolygon_node;
begin
  // TriStripList is global ... TriStripListLast points to last element in that list
  // old Manchester code replaced ....
  MALLOC(pointer(tn), sizeof(Tpolygon_node), 'tristrip node creation');
  tn.next := nil;
  tn.v[LEFT] := nil;
  tn.v[RIGHT] := nil;
  tn.lastv[LEFT] := nil;
  tn.lastv[RIGHT] := nil;
  tn.active := 0;
  Tadd_vertexL(tn, x, y); // active count  will be set to 1 here
  edge.outp[ABOVE] := tn;

  if (TriStripList <> nil) then
  begin
    TriStripListLast.next := tn;
    TriStripListLast := tn;
  end
  else
  begin
    TriStripList := tn;
    TriStripListLast := tn;
  end;
end;

function create_contour_bboxes(p: Pgpc_polygon): PbboxArray;
var
  c, v: integer;
begin
  MALLOC(pointer(result), p.num_contours * sizeof(Tbbox),
    'Bounding box creation');

  { Construct contour bounding boxes }
  for c := 0 to p.num_contours - 1 do
  begin
    { Initialise bounding box extent }
    result[c].xmin := DBL_MAX;
    result[c].ymin := DBL_MAX;
    result[c].xmax := -DBL_MAX;
    result[c].ymax := -DBL_MAX;

    for v := 0 to p.contour[c].num_vertices - 1 do
    begin
      { Adjust bounding Result }
      if (p.contour[c].vertex[v].x < result[c].xmin) then
        result[c].xmin := p.contour[c].vertex[v].x;
      if (p.contour[c].vertex[v].y < result[c].ymin) then
        result[c].ymin := p.contour[c].vertex[v].y;
      if (p.contour[c].vertex[v].x > result[c].xmax) then
        result[c].xmax := p.contour[c].vertex[v].x;
      if (p.contour[c].vertex[v].y > result[c].ymax) then
        result[c].ymax := p.contour[c].vertex[v].y;
    end;
  end;
end;

procedure minimax_test(subj: Pgpc_polygon; clip: Pgpc_polygon; op: Tgpc_op);
var
  s_bbox, c_bbox: PbboxArray;
  s, c: integer;
  o_table: PIntegerArray;
  overlap: integer;
begin // only for diff and int operations...
  s_bbox := create_contour_bboxes(subj);
  c_bbox := create_contour_bboxes(clip);

  MALLOC(pointer(o_table), subj.num_contours * clip.num_contours *
    sizeof(integer), 'overlap table creation');

  { Check all subject contour bounding boxes against clip boxes }
  for s := 0 to subj.num_contours - 1 do
    for c := 0 to clip.num_contours - 1 do
      o_table[c * subj.num_contours + s] :=
        integer((not((s_bbox[s].xmax < c_bbox[c].xmin) or
        (s_bbox[s].xmin > c_bbox[c].xmax))) and
        (not((s_bbox[s].ymax < c_bbox[c].ymin) or
        (s_bbox[s].ymin > c_bbox[c].ymax))));

  { For each clip contour, search for any subject contour overlaps }
  for c := 0 to clip.num_contours - 1 do
  begin
    overlap := 0;
    s := 0;
    while (overlap = 0) and (s < subj.num_contours) do
    begin
      overlap := o_table[c * subj.num_contours + s];
      Inc(s);
    end;

    if overlap = 0 then
      { Flag non contributing status by negating vertex count }
      clip.contour[c].num_vertices := -clip.contour[c].num_vertices;
  end;

  if (op = GPC_INT) then
  begin
    { For each subject contour, search for any clip contour overlaps }
    for s := 0 to subj.num_contours - 1 do
    begin
      overlap := 0;
      c := 0;
      while (overlap = 0) and (c < clip.num_contours) do
      begin
        overlap := o_table[c * subj.num_contours + s];
        Inc(c);
      end;

      if overlap = 0 then
        { Flag non contributing status by negating vertex count }
        subj.contour[s].num_vertices := -subj.contour[s].num_vertices;
    end;
  end;

  Free(pointer(s_bbox));
  Free(pointer(c_bbox));
  Free(pointer(o_table));
end;


// ===========================================================================
// Public Functions
// ===========================================================================

procedure gpc_free_polygon(polygon: Pgpc_polygon);
var
  c: integer;
begin
  for c := 0 to polygon.num_contours - 1 do
    CFree(pointer(polygon.contour[c].vertex));

  CFree(pointer(polygon.hole));
  CFree(pointer(polygon.contour));
  polygon.num_contours := 0;
end;

procedure gpc_read_polygon(var f: text; read_hole_flags: integer;
  p: Pgpc_polygon);
var
  c, v: integer;
begin
  readln(f, p.num_contours);
  MALLOC(pointer(p.hole), p.num_contours * sizeof(integer),
    'hole flag array creation');
  MALLOC(pointer(p.contour), p.num_contours * sizeof(Tgpc_vertex_list),
    'contour creation');
  for c := 0 to p.num_contours - 1 do
  begin
    readln(f, p.contour[c].num_vertices);

    if (read_hole_flags = 1) then
      readln(f, p.hole[c])
    else
      p.hole[c] := FFALSE; // * Assume all contours to be external */

    MALLOC(pointer(p.contour[c].vertex), p.contour[c].num_vertices *
      sizeof(Tgpc_vertex), 'vertex creation');
    for v := 0 to p.contour[c].num_vertices - 1 do
    begin
      read(f, p.contour[c].vertex[v].x);
      readln(f, p.contour[c].vertex[v].y);
    end;
  end;
end;

procedure gpc_write_polygon(var f: text; write_hole_flags: integer;
  p: Pgpc_polygon);
var
  c, v: integer;
begin
  writeln(f, p.num_contours);
  for c := 0 to p.num_contours - 1 do
  begin
    writeln(f, p.contour[c].num_vertices);

    if (write_hole_flags = 1) then
      writeln(f, p.hole[c]);

    for v := 0 to p.contour[c].num_vertices - 1 do
      writeln(f, p.contour[c].vertex[v].x:20:DBL_DIG, ' ',
        p.contour[c].vertex[v].y:20:DBL_DIG);
  end;
end;

procedure gpc_add_contour(polygon: Pgpc_polygon; contour: Pgpc_vertex_list;
  hole: integer);
var
  c, v: integer;
  extended_hole: PIntegerArray;
  extended_contour: Pgpc_vertex_list_array;
begin

  { Create an extended hole array }
  MALLOC(pointer(extended_hole), (polygon.num_contours + 1) * sizeof(integer),
    'contour hole addition');

  { Create an extended contour array }
  MALLOC(pointer(extended_contour), (polygon.num_contours + 1) *
    sizeof(Tgpc_vertex_list), 'contour addition');

  { Copy the old contour into the extended contour array }
  for c := 0 to polygon.num_contours - 1 do
  begin
    extended_hole[c] := polygon.hole[c];
    extended_contour[c] := polygon.contour[c];
  end;

  { Copy the new contour onto the end of the extended contour array }
  c := polygon.num_contours;
  extended_hole[c] := hole;
  extended_contour[c].num_vertices := contour.num_vertices;
  MALLOC(pointer(extended_contour[c].vertex), contour.num_vertices *
    sizeof(Tgpc_vertex), 'contour addition');
  for v := 0 to contour.num_vertices - 1 do
    extended_contour[c].vertex[v] := contour.vertex[v];

  { Dispose of the old contour }
  CFree(pointer(polygon.contour));
  CFree(pointer(polygon.hole));

  { Update the polygon information }
  Inc(polygon.num_contours);
  polygon.hole := extended_hole;
  polygon.contour := extended_contour;
end;

// ====================================================
function NiceEdgePair(var ipN: Pit_node): boolean;
Var
  { do NOT call with crooked ipN ,if (NOT ipN) return true! }
  t: boolean;
begin
  t := true;

  if (ipN <> nil) then
    t := (ipN.ie[0].next = ipN.bt[1]);

  NiceEdgePair := t;
end;

// ..copied form clipper pascal version ...
function lmCompare(Lm1, Lm2: pointer): integer;
var
  dy: double;
begin
  dy := Plmt_node(Lm2).y - Plmt_node(Lm1).y;
  if dy < 0 then
    lmCompare := 1
  else if dy > 0 then
    lmCompare := -1
  else
    lmCompare := 0;
end;

// ==============================
function EdgeisHorizontal(Cedge: Pedge_node): boolean;
begin // null is NOT horizontal here
  if (Cedge = nil) then
    EdgeisHorizontal := false
  else
    EdgeisHorizontal := (Cedge.bot.y = Cedge.top.y);
end;

procedure MurtaHorizontalSequence(Sedge: Pedge_node);
var
  HzEdge: Pedge_node;
  temp_edge: Pedge_node;
begin
  temp_edge := Sedge.succ;
  // patch to skip a Horizontal edge sequence,will take care of UP chain end
  while (EdgeisHorizontal(temp_edge)) do
    temp_edge := temp_edge.succ;
  if (temp_edge <> nil) then
  begin
    HzEdge := temp_edge;
    HzEdge.selnext := nil; // this is from aet next/prev???

    if (HzLast = nil) then
    begin
      HzLast := HzEdge;
      HzFirst := HzLast;
    end
    else
    begin
      HzLast.selnext := HzEdge;
      HzLast := HzEdge;
    end;

  end;
end; // Murta Horizontal Sequence

// =============================================
procedure PrepareForNextScanBeam();
var
  edge, Tmpedge: Pedge_node;
begin
  // Horizontal edge sequences are tackled only in forward chains..as of now...
  HzFirst := nil;
  HzLast := nil;
  edge := AET;
  while edge <> nil do
  begin
    next_edge := edge.next; // from aet
    prev_edge := edge.prev;
    succ_edge := edge.succ; // from edge table
    if ((edge.top.y = Yt) and (succ_edge <> nil)) then
    begin
      if (EdgeisHorizontal(succ_edge)) then
      begin
        MurtaHorizontalSequence(succ_edge);
        // collect non horizontal edge (if any) after the Murta sequence
        // and inject later  into aet with local minimas
        succ_edge := nil;
      end;
    end;
    if (edge.top.y = Yt) and (succ_edge <> nil) then
    begin
      { Replace AET edge by its successor }
      succ_edge.outp[BELOW] := edge.outp[ABOVE];
      succ_edge.bstate[BELOW] := edge.bstate[ABOVE];
      succ_edge.bundle[BELOW][clip] := edge.bundle[ABOVE][clip];
      succ_edge.bundle[BELOW][subj] := edge.bundle[ABOVE][subj];
      // prev_edge := edge.prev;
      UpdateScanLineQ(succ_edge.top.y); // top of new edge is a future scan line
      if prev_edge <> nil then
        prev_edge.next := succ_edge
      else
        AET := succ_edge;
      if next_edge <> nil then
        next_edge.prev := succ_edge;
      succ_edge.prev := prev_edge;
      succ_edge.next := next_edge;
    end
    else
    begin
      { Update this edge }
      edge.outp[BELOW] := edge.outp[ABOVE];
      edge.bstate[BELOW] := edge.bstate[ABOVE];
      edge.bundle[BELOW][clip] := edge.bundle[ABOVE][clip];
      edge.bundle[BELOW][subj] := edge.bundle[ABOVE][subj];
      edge.xb := edge.xt;
    end;
    edge.outp[ABOVE] := nil;
    edge := next_edge;
  end;

  // HorizontalsFollowEdgeToAET;

  HzEdge := HzFirst;
  while (HzEdge <> nil) do
  begin
    Tmpedge := HzEdge;
    add_edge_to_aet(AET, Tmpedge, nil); // they are inserted in x,dx order.!
    // if(TmpEdge <> nil)then
    UpdateScanLineQ(Tmpedge.top.y);
    HzEdge := HzEdge.selnext;
  end;
end;
procedure ReleaseTriStrips(); forward;

procedure EndCleanUp();
begin
  // * Tidy up */
  // ReleaseTriStrips();      Realease strips after conversion as triangles or contours
  FreeList(itList); // use
  FreeList(lm_List); // use dispose
  Free(pointer(c_heap));
  Free(pointer(s_heap));
  // Free(pointer(sbt));
end;

procedure GenerateResultPolygon(var result_polygon: Pgpc_polygon);
var
  poly, npoly: Ppolygon_node;
  vtx, nv: Pvertex_node;
  c, v: integer;
begin
  if result_polygon.num_contours > 0 then
  begin
    MALLOC(pointer(result_polygon.hole), result_polygon.num_contours *
      sizeof(integer), 'hole flag table creation');
    MALLOC(pointer(result_polygon.contour), result_polygon.num_contours *
      sizeof(Tgpc_vertex_list), 'contour creation');
    poly := out_poly;
    c := 0;
    // ================================
    while poly <> nil do
    begin
      npoly := poly.next;
      if poly.active > 0 then
      begin
        result_polygon.hole[c] := poly.hole;
        result_polygon.contour[c].num_vertices := poly.active;
        MALLOC(pointer(result_polygon.contour[c].vertex),
          result_polygon.contour[c].num_vertices * sizeof(Tgpc_vertex),
          'vertex creation');

        v := result_polygon.contour[c].num_vertices - 1;
        vtx := poly.v[LEFT];
        while vtx <> nil do
        begin
          nv := vtx.next;
          result_polygon.contour[c].vertex[v].x := vtx.x;
          result_polygon.contour[c].vertex[v].y := vtx.y;
          Free(pointer(vtx));
          Dec(v);
          vtx := nv;
        end;
        Inc(c);
      end;
      Free(pointer(poly));
      poly := npoly;
    end;
  end
  else
  begin
    poly := out_poly;
    while poly <> nil do
    begin
      npoly := poly.next;
      Free(pointer(poly));
      poly := npoly;
    end;

  end;
end;

function HrzRenderStep(edge: Pedge_node; var contributing: integer): integer;
var
  bl, br, tl, tr: integer;
  op: Tgpc_op;
begin
  op := clipop;
  tr := 0;
  tl := 0;
  br := 0;
  bl := 0;
  exists[gpc.clip] := edge.bundle[ABOVE][gpc.clip] +
    (edge.bundle[BELOW][gpc.clip] shl 1);
  exists[gpc.subj] := edge.bundle[ABOVE][gpc.subj] +
    (edge.bundle[BELOW][gpc.subj] shl 1);
  AELEDGEBelowH := edge; // communicate with  AETReplace
  contributing := FFALSE;

  if (exists[clip] <> 0) or (exists[subj] <> 0) then
  begin
    { Set bundle side }
    edge.bside[clip] := parity[clip];
    edge.bside[subj] := parity[subj];
    { Determine contributing status and quadrant occupancies }
    case op of
      GPC_DIFF, GPC_INT:
        begin
          contributing :=
            integer(((exists[clip] <> 0) and ((parity[subj] <> 0) or
            (horiz[subj] <> NH))) or ((exists[subj] <> 0) and
            ((parity[clip] <> 0) or (horiz[clip] <> NH))) or
            ((exists[clip] <> 0) and (exists[subj] <> 0) and
            (parity[clip] = parity[subj])));
          br := integer((parity[clip] <> 0) and (parity[subj] <> 0));
          bl := integer(((parity[clip] xor edge.bundle[ABOVE][clip]) <> 0) and
            ((parity[subj] xor edge.bundle[ABOVE][subj]) <> 0));
          tr := integer(((parity[clip] xor integer(horiz[clip] <> NH)) <> 0) and
            ((parity[subj] xor integer(horiz[subj] <> NH)) <> 0));
          tl := integer(((parity[clip] xor integer(horiz[clip] <> NH)
            xor edge.bundle[BELOW][clip]) <> 0) and
            ((parity[subj] xor integer(horiz[subj] <> NH) xor edge.bundle[BELOW]
            [subj]) <> 0));
        end;

      GPC_XOR:
        begin
          contributing := integer((exists[clip] <> 0) or (exists[subj] <> 0));
          br := integer(parity[clip] xor parity[subj]);
          bl := integer(((parity[clip] xor edge.bundle[ABOVE][clip]) <> 0)
            xor ((parity[subj] xor edge.bundle[ABOVE][subj]) <> 0));
          tr := integer(((parity[clip] xor integer(horiz[clip] <> NH)) <> 0)
            xor ((parity[subj] xor integer(horiz[subj] <> NH)) <> 0));
          tl := integer(((parity[clip] xor integer(horiz[clip] <> NH)
            xor edge.bundle[BELOW][clip]) <> 0)
            xor ((parity[subj] xor integer(horiz[subj] <> NH) xor edge.bundle
            [BELOW][subj]) <> 0));
        end;

      GPC_UNION:
        begin
          contributing :=
            integer(((exists[clip] <> 0) and ((parity[subj] = 0) or
            (horiz[subj] <> NH))) or ((exists[subj] <> 0) and
            ((parity[clip] = 0) or (horiz[clip] <> NH))) or
            ((exists[clip] <> 0) and (exists[subj] <> 0) and
            (parity[clip] = parity[subj])));

          br := integer((parity[clip] <> 0) or (parity[subj] <> 0));
          bl := integer(((parity[clip] xor edge.bundle[ABOVE][clip]) <> 0) or
            ((parity[subj] xor edge.bundle[ABOVE][subj]) <> 0));
          tr := integer(((parity[clip] xor integer(horiz[clip] <> NH)) <> 0) or
            ((parity[subj] xor integer(horiz[subj] <> NH)) <> 0));
          tl := integer(((parity[clip] xor integer(horiz[clip] <> NH)
            xor edge.bundle[BELOW][clip]) <> 0) or
            ((parity[subj] xor integer(horiz[subj] <> NH) xor edge.bundle[BELOW]
            [subj]) <> 0));
        end;
    end; { case }
  end; // exists...
  { Update parity }
  (* parity[CLIP] := integer((parity[CLIP] <> 0) xor (edge.bundle[ABOVE][CLIP] <> 0));
    parity[SUBJ] := integer((parity[SUBJ] <> 0) xor (edge.bundle[ABOVE][SUBJ] <> 0));
  *)

  parity[clip] := parity[clip] xor edge.bundle[ABOVE][clip];
  parity[subj] := parity[subj] xor edge.bundle[ABOVE][subj];

  { Update horizontal state }
  if exists[clip] <> 0 then
    horiz[clip] := next_h_state[integer(horiz[clip])
      ][((exists[clip] - 1) shl 1) + parity[clip]];

  if exists[subj] <> 0 then
    horiz[subj] := next_h_state[integer(horiz[subj])
      ][((exists[subj] - 1) shl 1) + parity[subj]];

  HrzRenderStep := tr + (tl shl 1) + (br shl 2) + (bl shl 3);
end;

function IntxRenderStep(e0, e1: Pedge_node): integer;
var
  bl, br, tl, tr: integer;
  op: Tgpc_op;
begin
  inn[clip] := integer(((e0.bundle[ABOVE][clip] <> 0) and (e0.bside[clip] = 0))
    or ((e1.bundle[ABOVE][clip] <> 0) and (e1.bside[clip] <> 0)) or
    ((e0.bundle[ABOVE][clip] = 0) and (e1.bundle[ABOVE][clip] = 0) and
    (e0.bside[clip] <> 0) and (e1.bside[clip] <> 0)));

  inn[subj] := integer(((e0.bundle[ABOVE][subj] <> 0) and (e0.bside[subj] = 0))
    or ((e1.bundle[ABOVE][subj] <> 0) and (e1.bside[subj] <> 0)) or
    ((e0.bundle[ABOVE][subj] = 0) and (e1.bundle[ABOVE][subj] = 0) and
    (e0.bside[subj] <> 0) and (e1.bside[subj] <> 0)));
  op := clipop;
  { Determine quadrant occupancies }
  case op of

    GPC_DIFF, GPC_INT:
      begin
        tr := integer((inn[clip] <> 0) and (inn[subj] <> 0));
        tl := integer(((inn[clip] xor e1.bundle[ABOVE][clip]) <> 0) and
          ((inn[subj] xor e1.bundle[ABOVE][subj]) <> 0));
        br := integer(((inn[clip] xor e0.bundle[ABOVE][clip]) <> 0) and
          ((inn[subj] xor e0.bundle[ABOVE][subj]) <> 0));
        bl := integer(((inn[clip] xor e1.bundle[ABOVE][clip] xor e0.bundle
          [ABOVE][clip]) <> 0) and
          ((inn[subj] xor e1.bundle[ABOVE][subj] xor e0.bundle[ABOVE]
          [subj]) <> 0));
      end;

    GPC_XOR:
      begin
        tr := integer((inn[clip] <> 0) xor (inn[subj] <> 0));
        tl := integer((inn[clip] xor e1.bundle[ABOVE][clip])
          xor (inn[subj] xor e1.bundle[ABOVE][subj]));
        br := integer((inn[clip] xor e0.bundle[ABOVE][clip])
          xor (inn[subj] xor e0.bundle[ABOVE][subj]));
        bl := integer((inn[clip] xor e1.bundle[ABOVE][clip] xor e0.bundle[ABOVE]
          [clip]) xor (inn[subj] xor e1.bundle[ABOVE][subj] xor e0.bundle
          [ABOVE][subj]));
      end;
    GPC_UNION:
      begin
        tr := integer((inn[clip] <> 0) or (inn[subj] <> 0));
        tl := integer(((inn[clip] xor e1.bundle[ABOVE][clip]) <> 0) or
          ((inn[subj] xor e1.bundle[ABOVE][subj]) <> 0));
        br := integer(((inn[clip] xor e0.bundle[ABOVE][clip]) <> 0) or
          ((inn[subj] xor e0.bundle[ABOVE][subj]) <> 0));
        bl := integer(((inn[clip] xor e1.bundle[ABOVE][clip] xor e0.bundle
          [ABOVE][clip]) <> 0) or
          ((inn[subj] xor e1.bundle[ABOVE][subj] xor e0.bundle[ABOVE]
          [subj]) <> 0));
      end;
  end; { case }

  IntxRenderStep := tr + (tl shl 1) + (br shl 2) + (bl shl 3);
end;

// ========================================================================
procedure PolyHRZRender(RCmd: integer; edge: Pedge_node; var cf: Ppolygon_node);
var
  contributing: integer;
  xb: double;
  q: Ppolygon_node;
  // cf:Ppolygon_node;    //carry over from step to step
  _class: integer;
begin
  // _class := HrzRenderStep(edge, contributing);
  // if contributing <> 0 then
  begin
    _class := RCmd;
    xb := edge.xb;
    q := edge.outp[BELOW];
    // Writeln(VattiMessages,'PH',_class:2);
    case Tvertex_type(_class) of
      EMN, IMN:
        begin
          add_local_min(edge, xb, Yb);
          cf := edge.outp[ABOVE];
        end;
      ERI:
        begin
          add_right(cf, xb, Yb);
          edge.outp[ABOVE] := cf;
          cf := nil;
        end;
      ELI:
        begin
          add_left(edge.outp[BELOW], xb, Yb);
          cf := edge.outp[BELOW];
          // edge.outp[BELOW] := nil; // --missing symmetry??
        end;
      EMX:
        begin
          add_left(cf, xb, Yb);
          merge_right(cf, q);
          edge.outp[BELOW] := nil;
          cf := nil;
        end;
      ILI:
        begin
          add_left(cf, xb, Yb);
          edge.outp[ABOVE] := cf;
          cf := nil;
        end;
      IRI:
        begin
          add_right(edge.outp[BELOW], xb, Yb);
          cf := edge.outp[BELOW];
          edge.outp[BELOW] := nil;
        end;
      IMX:
        begin
          add_right(cf, xb, Yb);
          merge_left(cf, q);
          edge.outp[BELOW] := nil;
          cf := nil;
        end;
      IMM:
        begin
          add_right(cf, xb, Yb);
          merge_left(cf, q);
          edge.outp[BELOW] := nil;
          add_local_min(edge, xb, Yb);
          cf := edge.outp[ABOVE];
        end;
      EMM:
        begin
          add_left(cf, xb, Yb);

          merge_right(cf, q);
          edge.outp[BELOW] := nil;
          add_local_min(edge, xb, Yb);
          cf := edge.outp[ABOVE];
        end;
      LED:
        begin
          if (edge.bot.y = Yb) then
            add_left(edge.outp[BELOW], xb, Yb);
          edge.outp[ABOVE] := edge.outp[BELOW];
          // edge.outp[BELOW] := nil; // missing symmetry
        end;
      RED:
        begin
          if (edge.bot.y = Yb) then
            add_right(edge.outp[BELOW], xb, Yb);
          edge.outp[ABOVE] := edge.outp[BELOW];
          // edge.outp[BELOW] := nil; // missing symmetry
        end;
    else
    end; { End of case }
  end; { End of contributing conditional }
end;

procedure PolyINTRender(RCmd: integer; Wintersect: Pit_node);
var
  e0, e1: Pedge_node;
  p, q: Ppolygon_node;
  prev_edge, next_edge: Pedge_node;
  vclass: integer;
  ix, iy: double;
begin

  e0 := Wintersect.ie[0];
  e1 := Wintersect.ie[1];
  p := e0.outp[ABOVE];
  q := e1.outp[ABOVE];
  ix := Wintersect.point.x;
  iy := Wintersect.point.y + Yb; // yb is global base of beam
  vclass := RCmd; // IntxRenderStep(e0, e1);
  // Writeln(VattiMessages,'PI',vclass:2);
  case Tvertex_type(vclass) of
    EMN:
      begin
        add_local_min(e0, ix, iy);
        e1.outp[ABOVE] := e0.outp[ABOVE];
      end;
    ERI:
      if p <> nil then
      begin
        add_right(p, ix, iy);
        e1.outp[ABOVE] := p;
        e0.outp[ABOVE] := nil;
      end;

    ELI:
      if q <> nil then
      begin
        add_left(q, ix, iy);
        e0.outp[ABOVE] := q;
        e1.outp[ABOVE] := nil;
      end;

    EMX:

      if (p <> nil) and (q <> nil) then
      begin
        add_left(p, ix, iy);
        e0.outp[ABOVE] := nil;
        merge_right(p, q);
        e1.outp[ABOVE] := nil;
      end;

    IMN:
      begin
        add_local_min(e0, ix, iy);
        e1.outp[ABOVE] := e0.outp[ABOVE];
      end;
    ILI:

      if p <> nil then
      begin
        add_left(p, ix, iy);
        e1.outp[ABOVE] := p;
        e0.outp[ABOVE] := nil;
      end;

    IRI:
      if q <> nil then
      begin
        add_right(q, ix, iy);
        e0.outp[ABOVE] := q;
        e1.outp[ABOVE] := nil;
      end;

    IMX:

      if (p <> nil) and (q <> nil) then
      begin
        add_right(p, ix, iy);
        e0.outp[ABOVE] := nil;
        merge_left(p, q);
        e1.outp[ABOVE] := nil;
      end;

    IMM:

      if (p <> nil) and (q <> nil) then
      begin
        add_right(p, ix, iy);
        e0.outp[ABOVE] := nil;
        merge_left(p, q);
        e1.outp[ABOVE] := nil;
        add_local_min(e0, ix, iy);
        e1.outp[ABOVE] := e0.outp[ABOVE];
      end;

    EMM:

      if (p <> nil) and (q <> nil) then
      begin
        add_left(p, ix, iy);
        e0.outp[ABOVE] := nil;
        merge_right(p, q);
        e1.outp[ABOVE] := nil;
        add_local_min(e0, ix, iy);
        e1.outp[ABOVE] := e0.outp[ABOVE];
      end;

  end; { End of case }

end;

// ========================================================================
procedure PAddChainEdgesAt(Yb: double; var local_min: Plmt_node;
  var Loc_min_idx: integer);
var
  edge: Pedge_node;
begin // identical for triangle Ccde
  { If LMT node corresponding to yb exists }
  if local_min <> nil then
  begin
    while (local_min.y = Yb) do
    begin
      { Add edges starting at this local minimum (==yb)to the AET }
      edge := local_min.Base_edge;
      add_edge_to_aet(AET, edge, nil); // they are inserted in x,dx order.!
      if (edge <> nil) then
        UpdateScanLineQ(edge.top.y);
      // top of edge becomes a candidate for a scan line
      if (Loc_min_idx = lm_List.Count) then
        break;
      local_min := lm_List[Loc_min_idx];
      Inc(Loc_min_idx);
    end;
    if (Loc_min_idx < lm_List.Count) then
      UpdateScanLineQ(local_min.y);
  end;
  // ===============================================
end;

procedure Initgpcclip(set_operation: Tgpc_op; subject_polygon: Pgpc_polygon;
  clip_polygon: Pgpc_polygon; result_polygon: Pgpc_polygon);

var // NOT Used
  lmt, local_min: Plmt_node;
  Loc_min_idx: integer;

  BaseLineY: double; // First scan line ...
  TerminalLineY: double;


  // sbtree: Psb_tree;      //used to keep unique Y values of the vertices
  // both subject,clip contours

begin
  AET := nil;
  clipop := set_operation;
  lmt := nil;
  out_poly := nil;
  // cf := nil;
  inn[0] := LEFT;
  inn[1] := LEFT;

  exists[0] := LEFT;
  exists[1] := LEFT;

  // scanbeam := 0;

  c_heap := nil;
  s_heap := nil;
  // AssignFile(VattiMessages,'Polymesag.txt');
  // ReWrite(VattiMessages);
  { Test for trivial NULL result cases }
  if ((subject_polygon.num_contours = 0) and (clip_polygon.num_contours = 0)) or
    ((subject_polygon.num_contours = 0) and ((set_operation = GPC_INT) or
    (set_operation = GPC_DIFF))) or ((clip_polygon.num_contours = 0) and
    (set_operation = GPC_INT)) then
  begin
    result_polygon.num_contours := 0;
    result_polygon.hole := nil;
    result_polygon.contour := nil;
    exit;
  end;

  { Identify potentialy contributing contours }
  if (((set_operation = GPC_INT) or (set_operation = GPC_DIFF)) and
    (subject_polygon.num_contours > 0) and (clip_polygon.num_contours > 0)) then
    minimax_test(subject_polygon, clip_polygon, set_operation);

  { Build LMT }
  sbt_Minimum := DBL_MAX;
  sbt_Maximum := -DBL_MAX;

  lm_List := TList.Create();
  lm_List.Clear;
  Lmt_NextFreeIndex := 0;
  Loc_min_idx := 0;
  LmtList_NextFreeIndex := 0;
  itList := TList.Create(); // list for intersections within a scan beam
  itList.Clear;
  if subject_polygon.num_contours > 0 then
    s_heap := build_lmt(lmt, subject_polygon, subj, set_operation);
  if clip_polygon.num_contours > 0 then
    c_heap := build_lmt(lmt, clip_polygon, clip, set_operation);
  lm_List.Sort(lmCompare);
  { Return a NULL result if no contours contribute }
  // if lmt = nil then
  if LmtList_NextFreeIndex = 0 then

  begin
    result_polygon.num_contours := 0;
    result_polygon.hole := nil;
    result_polygon.contour := nil;
    // reset_lmt(lmt);
    Free(pointer(s_heap));
    Free(pointer(c_heap));
    FreeList(lm_List); // release all new emory of list onjects
    exit;
  end;

  { Build scanbeam table from scanbeam tree }
  // MALLOC(pointer(sbt), sbt_entries * sizeof(double), 'sbt creation');
  // build_sbt(scanbeam, sbt, sbtree);

  UpdateScanLineQ(DBL_MAX); // sentinal  for scan lines
  // scanbeam := 0;
  // free_sbtree(sbtree);

  { Allow pointer re-use without causing memory leak }
  if subject_polygon = result_polygon then
    gpc_free_polygon(subject_polygon);
  if clip_polygon = result_polygon then
    gpc_free_polygon(clip_polygon);

  parity[0] := LEFT;
  parity[1] := LEFT;
  { Invert clip polygon for difference operation }
  if set_operation = GPC_DIFF then
    parity[clip] := RIGHT;

  /// / local_min := lmt;
  // sort the lmt on y,x,dx  ....
  // BaseLineY:=lmt.y;  //first minima !!

  /// /Keep  local_min := lmt;//_List[Loc_min_idx];
  Loc_min_idx := 0;
  // list out the lmt....
  local_min := lm_List[0];
  Inc(Loc_min_idx);
  BaseLineY := local_min.y;
  UpdateScanLineQ(BaseLineY); // 1st scan line
  itList := TList.Create(); // list for intersections within a scan beam
  itList.Clear;
  { Process each scanbeam }
  // exit; //only for timing ...{ 1.3 secs till this spot !!,1 sec for lmt! }
  // yt:=sbt[scanbeam];  //assumes sbt populated
  PopScanLineQ(Yt);
end;

// ==========================================
procedure gpc_polygon_clip(set_operation: Tgpc_op;
  subject_polygon: Pgpc_polygon; clip_polygon: Pgpc_polygon;
  result_polygon: Pgpc_polygon);

var
  // sbtree: Psb_tree;      //used to keep unique Y values of the vertices
  // both subject,clip contours
  it, intersect, Wintersect, PWintersect, Tmpintersect: Pit_node;
  edge: Pedge_node;
  e0, e1: Pedge_node;

  lmt, local_min: Plmt_node;
  Loc_min_idx: integer;
  // p, q, poly, npoly
  cf: Ppolygon_node;

  contributing, search, scanbeam: integer;
  sbt_entries: integer;
  vclass, bl, br, tl, tr: integer;
  sbt: PDoubleArray;
  xb, px, ix, iy: double;
  // yb, yt,dy:double;
  pt: boolean;
  BaseLineY: double; // First scan line ...
  TerminalLineY: double;

  Tmpedge: Pedge_node;

begin
  edge := nil;
  // sbtree := nil;
  it := nil;
  AET := nil;
  clipop := set_operation;
  lmt := nil;
  out_poly := nil;
  // cf := nil;
  inn[0] := LEFT;
  inn[1] := LEFT;
  exists[0] := LEFT;
  exists[1] := LEFT;

  // scanbeam := 0;
  sbt_entries := 0;
  sbt := nil;
  c_heap := nil;
  s_heap := nil;
  // AssignFile(VattiMessages,'Polymesag.txt');
  // ReWrite(VattiMessages);
  { Test for trivial NULL result cases }
  if ((subject_polygon.num_contours = 0) and (clip_polygon.num_contours = 0)) or
    ((subject_polygon.num_contours = 0) and ((set_operation = GPC_INT) or
    (set_operation = GPC_DIFF))) or ((clip_polygon.num_contours = 0) and
    (set_operation = GPC_INT)) then
  begin
    result_polygon.num_contours := 0;
    result_polygon.hole := nil;
    result_polygon.contour := nil;
    exit;
  end;

  { Identify potentialy contributing contours }
  if (((set_operation = GPC_INT) or (set_operation = GPC_DIFF)) and
    (subject_polygon.num_contours > 0) and (clip_polygon.num_contours > 0)) then
    minimax_test(subject_polygon, clip_polygon, set_operation);

  { Build LMT }
  sbt_Minimum := DBL_MAX;
  sbt_Maximum := -DBL_MAX;

  lm_List := TList.Create();
  lm_List.Clear;
  Lmt_NextFreeIndex := 0;
  Loc_min_idx := 0;
  LmtList_NextFreeIndex := 0;
  itList := TList.Create(); // list for intersections within a scan beam
  itList.Clear;
  if subject_polygon.num_contours > 0 then
    s_heap := build_lmt(lmt, subject_polygon, subj, set_operation);
  if clip_polygon.num_contours > 0 then
    c_heap := build_lmt(lmt, clip_polygon, clip, set_operation);
  lm_List.Sort(lmCompare);
  { Return a NULL result if no contours contribute }
  // if lmt = nil then
  if LmtList_NextFreeIndex = 0 then

  begin
    result_polygon.num_contours := 0;
    result_polygon.hole := nil;
    result_polygon.contour := nil;
    // reset_lmt(lmt);
    Free(pointer(s_heap));
    Free(pointer(c_heap));
    FreeList(lm_List); // release all new emory of list onjects
    exit;
  end;

  { Build scanbeam table from scanbeam tree }
  // MALLOC(pointer(sbt), sbt_entries * sizeof(double), 'sbt creation');
  // build_sbt(scanbeam, sbt, sbtree);

  UpdateScanLineQ(DBL_MAX); // sentinal  for scan lines
  // scanbeam := 0;
  // free_sbtree(sbtree);

  { Allow pointer re-use without causing memory leak }
  if subject_polygon = result_polygon then
    gpc_free_polygon(subject_polygon);
  if clip_polygon = result_polygon then
    gpc_free_polygon(clip_polygon);

  parity[0] := LEFT;
  parity[1] := LEFT;
  { Invert clip polygon for difference operation }
  if set_operation = GPC_DIFF then
    parity[clip] := RIGHT;

  /// / local_min := lmt;
  // sort the lmt on y,x,dx  ....
  // BaseLineY:=lmt.y;  //first minima !!

  /// /Keep  local_min := lmt;//_List[Loc_min_idx];
  Loc_min_idx := 0;
  // list out the lmt....
  local_min := lm_List[0];
  Inc(Loc_min_idx);
  BaseLineY := local_min.y;
  UpdateScanLineQ(BaseLineY); // 1st scan line
  itList := TList.Create(); // list for intersections within a scan beam
  itList.Clear;
  { Process each scanbeam }
  // exit; //only for timing ...{ 1.3 secs till this spot !!,1 sec for lmt! }
  // yt:=sbt[scanbeam];  //assumes sbt populated
  PopScanLineQ(Yt);

  while (Yt <= sbt_Maximum) do
  begin
    { Set yb and yt to the bottom and top of the scanbeam }
    Yb := Yt;

    { === SCANBEAM BOUNDARY PROCESSING ================================ }
    { If LMT node corresponding to yb exists }
    PAddChainEdgesAt(Yb, local_min, Loc_min_idx); // same as for strip code

    // Inc(scanbeam);
    // if scanbeam < sbt_entries then
    if (Yt < sbt_Maximum) then
    begin

      PopScanLineQ(Yt); // why is this init not taking place???
      // yt := sbt[scanbeam];
    end;


    // dy := yt - yb;

    { Set dummy previous x value }
    // px := -DBL_MAX;

    { Create bundles within AET }
    e0 := AET;
    e1 := AET;

    { Set up bundle fields of first edge }
    AET.bundle[ABOVE][integer(AET.typ <> 0)] := integer((AET.top.y <> Yb));
    AET.bundle[ABOVE][integer(AET.typ = 0)] := FFALSE;
    AET.bstate[ABOVE] := UNBUNDLED;

    next_edge := AET.next;

    while next_edge <> nil do
    begin
      { Set up bundle fields of next edge }
      next_edge.bundle[ABOVE][next_edge.typ] :=
        integer((next_edge.top.y <> Yb));
      next_edge.bundle[ABOVE][integer(next_edge.typ = 0)] := FFALSE;
      next_edge.bstate[ABOVE] := UNBUNDLED;

      { Bundle edges above the scanbeam boundary if they coincide }
      if next_edge.bundle[ABOVE][next_edge.typ] <> 0 then
      begin
        if (EQ(e0.xb, next_edge.xb) and EQ(e0.dx, next_edge.dx) and
          (e0.top.y <> Yb)) then
        begin
          next_edge.bundle[ABOVE][next_edge.typ] := next_edge.bundle[ABOVE]
            [next_edge.typ] xor e0.bundle[ABOVE][next_edge.typ];
          next_edge.bundle[ABOVE][integer(next_edge.typ = 0)] :=
            e0.bundle[ABOVE][integer(next_edge.typ = 0)];
          next_edge.bstate[ABOVE] := BUNDLE_HEAD;
          e0.bundle[ABOVE][clip] := FFALSE;
          e0.bundle[ABOVE][subj] := FFALSE;
          e0.bstate[ABOVE] := BUNDLE_TAIL;
        end;
        e0 := next_edge;
      end;
      next_edge := next_edge.next;
    end;

    horiz[clip] := NH;
    horiz[subj] := NH;
    // init parity here fresh if not sure...
    { Process each edge at this scanbeam boundary }
    edge := AET;
    cf := nil;
    while edge <> nil do
    begin
      exists[clip] := edge.bundle[ABOVE][clip] +
        (edge.bundle[BELOW][clip] shl 1);
      exists[subj] := edge.bundle[ABOVE][subj] +
        (edge.bundle[BELOW][subj] shl 1);
      AELEDGEBelowH := edge; // communicate with global AELReplace
      if (exists[clip] <> 0) or (exists[subj] <> 0) then
      begin
        vclass := HrzRenderStep(edge, contributing);
        if contributing <> 0 then
          PolyHRZRender(vclass, edge, cf);
      end; { End of edge exists conditional }

      edge := edge.next;
    end; { End of AET loop }
    // =================Horizontal========================================
    { Delete terminating edges from the AET, otherwise compute xt }
    edge := AET;
    while edge <> nil do
    begin
      if (edge.top.y = Yb) then
      begin
        prev_edge := edge.prev;
        next_edge := edge.next;
        if prev_edge <> nil then
          prev_edge.next := next_edge
        else
          AET := next_edge;
        if next_edge <> nil then
          next_edge.prev := prev_edge;

        { Copy bundle head state to the adjacent tail edge if required }
        if (edge.bstate[BELOW] = BUNDLE_HEAD) and (prev_edge <> nil) then
        begin
          if prev_edge.bstate[BELOW] = BUNDLE_TAIL then
          begin
            prev_edge.outp[BELOW] := edge.outp[BELOW];
            prev_edge.bstate[BELOW] := UNBUNDLED;
            if prev_edge.prev <> nil then
              if prev_edge.prev.bstate[BELOW] = BUNDLE_TAIL then
                prev_edge.bstate[BELOW] := BUNDLE_HEAD;
          end;
        end;
      end
      else
      begin
        if (edge.top.y = Yt) then
          edge.xt := edge.top.x
        else
          edge.xt := edge.bot.x + edge.dx * (Yt - edge.bot.y);
      end;

      edge := edge.next;
    end;
    // if yb == yt aet should be empty
    if Yb <> Yt then
    begin
      { === SCANBEAM INTERIOR PROCESSING ============================== }

      build_intersection_table(AET); // , yt-yb);
      { Get Tails for the intersecting bundles for resequencing The bundles }
      // itList sorted on y point of intersection
      // already done above  TailForIntersectionNodeEdges(it);
      Yt := Yt;
      { Process each node in the intersection table }
      intersect := nil;
      if (itList.Count > 0) then
        intersect := itList[0];
      Tmpintersect := nil;
      AELEDGEBelowH := nil; // communicate with global AELReplace..
      if (itList.Count > 0) then
        while intersect <> nil do
        begin
          { get a nice intersection }
          Wintersect := intersect;
          Tmpintersect := nil;
          if NOT NiceEdgePair(Wintersect) then
          begin
            Tmpintersect := Wintersect;
            while (NOT NiceEdgePair(Wintersect)) do
            begin
              PWintersect := Wintersect;
              Wintersect := Wintersect.next;
            end;
            PWintersect.next := Wintersect.next;
          end;

          e0 := Wintersect.ie[0];
          e1 := Wintersect.ie[1];

          { ===very Important e0 bundle and e1 bundle should be adjacent!
            if NOT get one which passes the assertion !
            i.e.  Use The ELDER WAND !,
          }

          { Only generate output for contributing intersections }
          if ((e0.bundle[ABOVE][clip] <> 0) or (e0.bundle[ABOVE][subj] <> 0))
            and ((e1.bundle[ABOVE][clip] <> 0) or (e1.bundle[ABOVE][subj] <> 0))
          then
          begin
            vclass := IntxRenderStep(e0, e1);
            PolyINTRender(vclass, Wintersect);
          end; { End of contributing intersection conditional }

          { Swap bundle sides in response to edge crossing }
          if (e0.bundle[ABOVE][clip] <> 0) then
            e1.bside[clip] := integer(e1.bside[clip] = 0);
          if (e1.bundle[ABOVE][clip] <> 0) then
            e0.bside[clip] := integer(e0.bside[clip] = 0);
          if (e0.bundle[ABOVE][subj] <> 0) then
            e1.bside[subj] := integer(e1.bside[subj] = 0);
          if (e1.bundle[ABOVE][subj] <> 0) then
            e0.bside[subj] := integer(e0.bside[subj] = 0);

          { Swap e0 and e1 bundles in the AET }
          prev_edge := e0.prev;
          next_edge := e1.next;
          if next_edge <> nil then
            next_edge.prev := e0;
          { The code below assumes e0 and e1 are adjacent bundles }
          if e0.bstate[ABOVE] = BUNDLE_HEAD then
          begin
            search := FTRUE;
            while search <> 0 do
            begin
              prev_edge := prev_edge.prev;
              if prev_edge <> nil then
              begin
                if prev_edge.bstate[ABOVE] <> BUNDLE_TAIL then
                  search := FFALSE;
              end
              else
                search := FFALSE;
            end;
          end;
          if prev_edge = nil then
          begin
            AET.prev := e1;
            e1.next := AET;
            AET := e0.next;
          end
          else
          begin
            prev_edge.next.prev := e1;
            e1.next := prev_edge.next;
            prev_edge.next := e0.next;
          end;
          e0.next.prev := prev_edge;
          e1.next.prev := e1;
          e0.next := next_edge;

          { get next  intersection }
          if (Tmpintersect <> nil) then
          begin
            intersect := Tmpintersect;
            Tmpintersect := nil;
          end
          else
            intersect := intersect.next;
        end; { End of IT loop }
      if (itList.Count > 0) then
        itList.Clear();
      // FreeList(itList);
      // =====================================
      { Prepare for next scanbeam }
      PrepareForNextScanBeam();
      // Horizontal edge sequences are tackled only as forward edges.
    end; // if (yb <> yt)

    if (Yb = Yt) then
      break;

  end; { === END OF SCANBEAM PROCESSING ================================== }

  FreeScanLineQ();

  { Generate result polygon from out_poly }
  result_polygon.contour := nil;
  result_polygon.hole := nil;
  result_polygon.num_contours := 0;
  result_polygon.num_contours := count_contours(out_poly);
  GenerateResultPolygon(result_polygon);
  EndCleanUp();
  // CloseFile(vattiMessages);
end;

// ===========================================================
procedure gpc_free_tristrip(tristrip: Pgpc_tristrip);
var
  s: integer;
begin
  for s := 0 to tristrip.num_strips - 1 do
    CFree(pointer(tristrip.strip[s].vertex));
  CFree(pointer(tristrip.strip));
  tristrip.num_strips := 0;
end;

procedure gpc_polygon_to_tristrip(s: Pgpc_polygon; t: Pgpc_tristrip);
var
  c: Tgpc_polygon;
begin
  c.num_contours := 0;
  c.hole := nil;
  c.contour := nil;
  gpc_tristrip_clip(GPC_DIFF, s, @c, t);

end;

// =========================================================
function CountStripvertices(lt, rt: Pvertex_node): integer;
var
  Lcnt, Rcnt: integer;
  ltn, rtn: Pvertex_node;
begin

  Lcnt := 0;
  ltn := lt;
  while (ltn <> nil) do
  begin
    Inc(Lcnt);
    ltn := ltn.next;
  end;

  Rcnt := 0;
  rtn := rt;
  while (rtn <> nil) do
  begin
    Inc(Rcnt);
    rtn := rtn.next;
  end;

  result := Lcnt;
  if (Lcnt < Rcnt) then
    result := Rcnt;
  result := 2 * result;
  CountStripvertices := result;
end;
 //===================================
procedure GenerateTriStrips(result: Pgpc_tristrip);
var
  s, v: integer;
  ltx, lty, rtx, rty: double;
  ltn, rtn: Pvertex_node;
  stripvertices: integer;
begin
//Murta's recipe does NOT work for unmatching arm lengths   so leave the recipe to rot ....
  // The raw arms are preserved here ,in case
  //some different processing is to be done later.!
  result.strip := nil;
  result.num_strips := count_tristrips(TriStripList);
  if (result.num_strips > 0) then
  begin
    MALLOC(pointer(result.strip), result.num_strips * sizeof(Tgpc_vertex_list),
      'tristrip list creation');
    s := 0;
    tn := TriStripList;
    while (tn <> nil) do
    begin
      tnn := tn.next;

      if (tn.active > 2) then
      begin
        // * Valid tristrip: copy the vertices and free the heap */
        // number  of vertices in strip is 2*max(leftvertices,rightvertices)

        if (INVERT_TRISTRIPS <> 0) then
        begin
          lt := tn.v[RIGHT];
          rt := tn.v[LEFT];
        end
        else
        begin
          lt := tn.v[LEFT];
          rt := tn.v[RIGHT];
        end;

        // result.strip[s].num_vertices := tn.active;
        stripvertices := CountStripvertices(lt, rt);
        result.strip[s].num_vertices := stripvertices;
        MALLOC(pointer(result.strip[s].vertex),
          stripvertices * sizeof(Tgpc_vertex), 'tristrip creation');
        v := 0;
        while ((lt <> nil) or (rt <> nil)) do
        begin
          if (lt <> nil) then
          begin
            ltx := lt.x;
            lty := lt.y;
            // result.strip[s].vertex[v].x := lt.x;
            // result.strip[s].vertex[v].y := lt.y;
            // Inc(v);
            ltn := lt.next;
            // Free(pointer(lt));
            lt := ltn;
          end;
          result.strip[s].vertex[v].x := ltx;
          result.strip[s].vertex[v].y := lty;
          Inc(v);
          if (rt <> nil) then
          begin
            rtx := rt.x;
            rty := rt.y;
            // result.strip[s].vertex[v].x := rt.x;
            // result.strip[s].vertex[v].y := rt.y;
            // Inc(v);
            rtn := rt.next;
            // Free(pointer(rt));
            rt := rtn;
          end;
          result.strip[s].vertex[v].x := rtx;
          result.strip[s].vertex[v].y := rty;
          Inc(v);
        end;
        Inc(s);
      end
      else if (false) then
      begin
        // * Invalid tristrip: just free the heap */
        lt := tn.v[LEFT];
        while (lt <> nil) do
        begin
          ltn := lt.next;
          // Free(pointer(lt));
          lt := ltn
        end;
        rt := tn.v[RIGHT];
        while (rt <> nil) do
        begin
          rtn := rt.next;
          // Free(pointer(rt));
          rt := rtn
        end;
      end;
      // tn.v[LEFT]:=nil;
      // tn.v[RIGHT]:=nil;
      // Free(pointer(tn));
      tn := tnn;
    end;
  end;
  // TriStripList:=nil;
end;

// =============================================================
procedure ReleaseTriStrips();
var
  // NOT working ..do not use....
  ltn, rtn: Pvertex_node;

begin

  tn := TriStripList;
  while (tn <> nil) do
  begin
    tnn := tn.next;
    lt := tn.v[LEFT];
    rt := tn.v[RIGHT];
    while (lt <> nil) do
    begin
      ltn := lt.next;
      Free(pointer(lt));
      lt := ltn;
    end;

    while (rt <> nil) do
    begin
      rtn := rt.next;
      Free(pointer(rt));
      rt := rtn;
    end;

    Free(pointer(tn));
    tn := tnn;
  end;
  TriStripList := nil;
end;

// ========================================================
procedure TriHRZRender(RCmd: integer; edge: Pedge_node; var cf: Pedge_node;
  var cft: Tvertex_type);
var
  // contributing: integer;
  xb: double;
  // cf:Pedge_node;    //carry over from step to step
  // cft:Tvertex_type; //carry over ......
  // vclass: integer;
begin

  // vclass := HrzRenderStep(edge, contributing);
  // if (contributing <> 0) then
  begin
    xb := edge.xb;
    // Writeln(VattiMessages,'PH',RCmd:2);
    case Tvertex_type(RCmd) of
      EMN:
        begin
          new_tristrip(edge, xb, Yb);
          cf := edge;
        end;
      ERI:
        begin
          edge.outp[ABOVE] := cf.outp[ABOVE];
          if (xb <> cf.xb) then
            vertexR(edge, ABOVE, xb, Yb);
          cf := nil;
        end;
      ELI:
        begin
          vertexL(edge, BELOW, xb, Yb);
          // nt 6/8/2016   edge.outp[ABOVE] := nil;
          cf := edge;
        end;
      EMX:
        begin

          // The scarce example exhibits this nicely   ..dramatic diiference ..
 //         if (xb <> cf.xb) then  // the routine cries for some xor
          vertexR(edge, BELOW, xb, Yb);
          edge.outp[ABOVE] := nil;
          cf := nil;
        end;
      IMN:
        begin
          if (cft = LED) then
          begin
            if (cf.bot.y <> Yb) then
              vertexL(cf, BELOW, cf.xb, Yb);
            new_tristrip(cf, cf.xb, Yb);
          end;
          edge.outp[ABOVE] := cf.outp[ABOVE];
          vertexR(edge, ABOVE, xb, Yb);
        end;
      ILI:
        begin
          new_tristrip(edge, xb, Yb);
          cf := edge;
          cft := ILI;
        end;
      IRI:
        begin
          if (cft = LED) then
          begin
            if (cf.bot.y <> Yb) then
              vertexL(cf, BELOW, cf.xb, Yb);
            new_tristrip(cf, cf.xb, Yb);
          end;
          vertexR(edge, BELOW, xb, Yb);
          edge.outp[ABOVE] := nil;
        end;
      IMX:
        begin
          vertexL(edge, BELOW, xb, Yb);
          edge.outp[ABOVE] := nil;
          cft := IMX;
        end;
      IMM:
        begin
          vertexL(edge, BELOW, xb, Yb);
          edge.outp[ABOVE] := cf.outp[ABOVE];
          if (xb <> cf.xb) then
            vertexR(cf, ABOVE, xb, Yb);
          cf := edge;
        end;
      EMM:
        begin
          vertexR(edge, BELOW, xb, Yb);
          edge.outp[ABOVE] := nil;
          new_tristrip(edge, xb, Yb);
          cf := edge;
        end;
      LED:
        begin
          if (edge.bot.y = Yb) then
            vertexL(edge, BELOW, xb, Yb);
          edge.outp[ABOVE] := edge.outp[BELOW];
          cf := edge;
          cft := LED;
        end;
      RED:
        begin
          edge.outp[ABOVE] := cf.outp[ABOVE];
          if (cft = LED) then
          begin
            if (cf.bot.y = Yb) then
              vertexR(edge, BELOW, xb, Yb)
            else if (edge.bot.y = Yb) then
            begin
              vertexL(cf, BELOW, cf.xb, Yb);
              vertexR(edge, BELOW, xb, Yb);
            end;
          end
          else
          begin
            vertexR(edge, BELOW, xb, Yb);
            vertexR(edge, ABOVE, xb, Yb);
          end;
          cf := nil;
        end;
      // * End of switch */
    end;
  end;
end;

procedure TriINTRender(RCmd: integer; Wintersect: Pit_node);
var // works for  all  4 ops...
  e0, e1: Pedge_node;
  p, q: Ppolygon_node;
  px, nx: double;
  prev_edge, next_edge: Pedge_node;
  vclass: integer;
  ix, iy: double; // px,nx are var parameters(?)
  // nx, px: double;
begin
  e0 := Wintersect.ie[0];
  e1 := Wintersect.ie[1];
  p := e0.outp[ABOVE];
  q := e1.outp[ABOVE];
  ix := Wintersect.point.x;
  iy := Wintersect.point.y + Yb;
  // vclass := IntxRenderStep(e0, e1);
  // Rcmd:=vclass;
  // Writeln(VattiMessages,'PI',RCmd:2);
  case Tvertex_type(RCmd) of
    EMN:
      begin
        new_tristrip(e1, ix, iy);
        e0.outp[ABOVE] := e1.outp[ABOVE];
      end;
    ERI:

      if (p <> nil) then
      begin
        P_EDGE(prev_edge, e0, ABOVE, px, iy);
        vertexL(prev_edge, ABOVE, px, iy);
        vertexR(e0, ABOVE, ix, iy);
        e1.outp[ABOVE] := e0.outp[ABOVE];
        e0.outp[ABOVE] := nil;
      end;

    ELI:

      if (q <> nil) then
      begin
        N_EDGE(next_edge, e1, ABOVE, nx, iy);
        // vertexL(e1, ABOVE, ix, iy);
        vertexR(next_edge, ABOVE, nx, iy);
        vertexL(e1, ABOVE, ix, iy);
        e0.outp[ABOVE] := e1.outp[ABOVE];
        e1.outp[ABOVE] := nil;
      end;

    EMX:

      if ((p <> nil) and (q <> nil)) then
      begin
        vertexL(e0, ABOVE, ix, iy);
        e0.outp[ABOVE] := nil;
        e1.outp[ABOVE] := nil;

      end;
    IMN:
      begin
        P_EDGE(prev_edge, e0, ABOVE, px, iy);
        vertexL(prev_edge, ABOVE, px, iy);
        N_EDGE(next_edge, e1, ABOVE, nx, iy);
        vertexR(next_edge, ABOVE, nx, iy);
        new_tristrip(prev_edge, px, iy);
        e1.outp[ABOVE] := prev_edge.outp[ABOVE];
        vertexR(e1, ABOVE, ix, iy);
        new_tristrip(e0, ix, iy);
        next_edge.outp[ABOVE] := e0.outp[ABOVE];
        vertexR(next_edge, ABOVE, nx, iy);
      end;
    ILI:

      if (p <> nil) then
      begin
        vertexL(e0, ABOVE, ix, iy);
        N_EDGE(next_edge, e1, ABOVE, nx, iy);
        vertexR(next_edge, ABOVE, nx, iy);
        e1.outp[ABOVE] := e0.outp[ABOVE];
        e0.outp[ABOVE] := nil;
      end;

    IRI:

      if (q <> nil) then
      begin
        vertexR(e1, ABOVE, ix, iy);
        P_EDGE(prev_edge, e0, ABOVE, px, iy);
        vertexL(prev_edge, ABOVE, px, iy);
        e0.outp[ABOVE] := e1.outp[ABOVE];
        e1.outp[ABOVE] := nil;
      end;

    IMX:

      if ((p <> nil) and (q <> nil)) then
      begin
        vertexR(e0, ABOVE, ix, iy);
        vertexL(e1, ABOVE, ix, iy);
        e0.outp[ABOVE] := nil;
        e1.outp[ABOVE] := nil;
        P_EDGE(prev_edge, e0, ABOVE, px, iy);
        vertexL(prev_edge, ABOVE, px, iy);
        new_tristrip(prev_edge, px, iy);
        N_EDGE(next_edge, e1, ABOVE, nx, iy);
        vertexR(next_edge, ABOVE, nx, iy);
        next_edge.outp[ABOVE] := prev_edge.outp[ABOVE];
        vertexR(next_edge, ABOVE, nx, iy);
      end;

    IMM:

      if ((p <> nil) and (q <> nil)) then
      begin
        vertexR(e0, ABOVE, ix, iy);
        vertexL(e1, ABOVE, ix, iy);
        P_EDGE(prev_edge, e0, ABOVE, px, iy);
        vertexL(prev_edge, ABOVE, px, iy);
        new_tristrip(prev_edge, px, iy);
        N_EDGE(next_edge, e1, ABOVE, nx, iy);
        vertexR(next_edge, ABOVE, nx, iy);
        e1.outp[ABOVE] := prev_edge.outp[ABOVE];
        vertexR(e1, ABOVE, ix, iy);
        new_tristrip(e0, ix, iy);
        next_edge.outp[ABOVE] := e0.outp[ABOVE];
        vertexR(next_edge, ABOVE, nx, iy);
      end;

    EMM:

      if ((p <> nil) and (q <> nil)) then
      begin
        vertexL(e0, ABOVE, ix, iy);
        new_tristrip(e1, ix, iy);
        e0.outp[ABOVE] := e1.outp[ABOVE];
      end;

  end; // * End of switch */
end;

// ====================================
procedure ProcessWithinBeamIntersections(Rtype: boolean);
var
  intersect, Wintersect, PWintersect, Tmpintersect: Pit_node;
  e0, e1: Pedge_node;
  search: integer;
  vclass: integer;

begin
  intersect := nil;
  // * === SCANBEAM INTERIOR PROCESSING ============================== */
  build_intersection_table(AET); // , yt-yb);
  { Get Tails for the intersecting bundles for resequencing The bundles }
  // itList sorted on y point of intersection
  // already done above  TailForIntersectionNodeEdges(it);
  { ===very Important e0 bundle and e1 bundle should be adjacent!
    if NOT get one which meets the constrain
    Use The ELDER WAND !,
  }
  if (itList.Count > 0) then
    intersect := itList[0];
  Tmpintersect := nil;
  AELEDGEBelowH := nil; // communicate with global AELReplace..
  if (itList.Count > 0) then
    while intersect <> nil do
    begin
      { get a nice intersection }
      Wintersect := intersect;
      if NOT NiceEdgePair(Wintersect) then
      begin
        Tmpintersect := Wintersect;
        while (NOT NiceEdgePair(Wintersect)) do
        begin
          PWintersect := Wintersect;
          Wintersect := Wintersect.next;
        end;
        PWintersect.next := Wintersect.next;
      end;
      // ============================
      // ====test for assertion of adjacency here=============
      e0 := Wintersect.ie[0];
      e1 := Wintersect.ie[1];
      // * Only generate output for contributing intersections */
      if (((e0.bundle[ABOVE][gpc.clip] <> 0) or (e0.bundle[ABOVE][gpc.subj] <>
        0)) and ((e1.bundle[ABOVE][gpc.clip] <> 0) or
        (e1.bundle[ABOVE][gpc.subj] <> 0))) then
      begin
        /// ======================get rendercode
        vclass := IntxRenderStep(e0, e1);
        if (Rtype) then
          PolyINTRender(vclass, Wintersect)
        else
          TriINTRender(vclass, Wintersect);
      end; // * End of contributing intersection conditional */
      // * Swap bundle sides in response to edge crossing */
      if (e0.bundle[ABOVE][gpc.clip] <> 0) then
        e1.bside[gpc.clip] := Ord(e1.bside[gpc.clip] = 0);
      if (e1.bundle[ABOVE][gpc.clip] <> 0) then
        e0.bside[gpc.clip] := Ord(e0.bside[gpc.clip] = 0);
      if (e0.bundle[ABOVE][gpc.subj] <> 0) then
        e1.bside[gpc.subj] := Ord(e1.bside[gpc.subj] = 0);
      if (e1.bundle[ABOVE][gpc.subj] <> 0) then
        e0.bside[gpc.subj] := Ord(e0.bside[gpc.subj] = 0);

      // * Swap e0 and e1 bundles in the AET
      // Requires e0->AELnext=e1*/
      prev_edge := e0.prev;
      next_edge := e1.next;
      if (e1.next <> nil) then
        e1.next.prev := e0;

      if (e0.bstate[ABOVE] = BUNDLE_HEAD) then
      begin
        search := FTRUE;
        while (search <> FFALSE) do
        begin
          prev_edge := prev_edge.prev;
          if (prev_edge <> nil) then
          begin
            if ((prev_edge.bundle[ABOVE][gpc.clip] <> 0) or
              (prev_edge.bundle[ABOVE][gpc.subj] <> 0) or
              (prev_edge.bstate[ABOVE] = BUNDLE_HEAD)) then
              search := FFALSE;
          end
          else
            search := FFALSE;
        end;
      end;
      if (prev_edge = nil) then
      begin
        e1.next := AET;
        AET := e0.next;
      end
      else
      begin
        e1.next := prev_edge.next;
        prev_edge.next := e0.next;
      end;
      e0.next.prev := prev_edge;
      e1.next.prev := e1;
      e0.next := next_edge;
      if (Tmpintersect <> nil) then
      begin
        intersect := Tmpintersect;
        Tmpintersect := nil;
      end
      else
        intersect := intersect.next;
    end; // * End of IT loop*/
  if (itList.Count > 0) then
    itList.Clear();
  // * Prepare for next scanbeam */
  // ===========================
  if (itList.Count > 0) then
    itList.Clear();
  // FreeList(itList);
end;

// ==========================================================
Procedure ProcessEdgesSpanningDividingLine(Rtype: boolean);
var
  edge: Pedge_node;
  Tricf: Pedge_node;
  Polycf: Ppolygon_node;
  cft: Tvertex_type;
  vclass: integer;
  contributing: integer;
  lmt, local_min: Plmt_node;
  tn, tnn, p, q: Ppolygon_node;

begin
  // * Process each edge at this scanbeam boundary */
  // Use Murtas 3 state predictor for horiz.....
  horiz[clip] := NH;
  horiz[subj] := NH;
  // op := clipop;
  exists[clip] := LEFT;
  exists[subj] := LEFT;
  parity[clip] := LEFT;
  parity[subj] := LEFT;
  if clipop = GPC_DIFF then
    parity[clip] := RIGHT;

  edge := AET;
  Tricf := nil;
  Polycf := nil;
  cft := NUL;
  while edge <> nil do
  begin
    exists[clip] := edge.bundle[ABOVE][clip] +
      (edge.bundle[BELOW][gpc.clip] shl 1);
    exists[subj] := edge.bundle[ABOVE][subj] +
      (edge.bundle[BELOW][gpc.subj] shl 1);
    // ====================================
    if (exists[clip] <> 0) or (exists[subj] <> 0) then
    begin
      vclass := HrzRenderStep(edge, contributing);
      // =================================
      if (contributing <> 0) then
      begin
        if (Rtype) then
          PolyHRZRender(vclass, edge, Polycf)
        else
          TriHRZRender(vclass, edge, Tricf, cft);
      end;
      // * End of edge exists conditional */
    end;
    // * End of AET loop */
    edge := edge.next
  end;
end;

procedure DeleteTerminatingEdgesFromAET();
var
  edge: Pedge_node;
begin
  edge := AET;
  while edge <> nil do
  begin
    if (edge.top.y = Yb) then
    begin
      prev_edge := edge.prev;
      next_edge := edge.next;
      if (prev_edge <> nil) then
        prev_edge.next := next_edge
      else
        AET := next_edge;
      if (next_edge <> nil) then
        next_edge.prev := prev_edge;

      // * Copy bundle head state to the adjacent tail edge if required */
      if ((edge.bstate[BELOW] = BUNDLE_HEAD) and (prev_edge <> nil)) then
      begin
        if (prev_edge.bstate[BELOW] = BUNDLE_TAIL) then
        begin
          prev_edge.outp[BELOW] := edge.outp[BELOW];
          prev_edge.bstate[BELOW] := UNBUNDLED;
          if (prev_edge.prev <> nil) then
            if (prev_edge.prev.bstate[BELOW] = BUNDLE_TAIL) then
              prev_edge.bstate[BELOW] := BUNDLE_HEAD;
        end;
      end;
    end
    else
    begin
      if (edge.top.y = Yt) then
        edge.xt := edge.top.x
      else
        edge.xt := edge.bot.x + edge.dx * (Yt - edge.bot.y);
    end;

    edge := edge.next
  end;
end;

procedure FormAETBundles();
var
  e0, e1: Pedge_node;
begin
  // * Create bundles within AET */
  e0 := AET;
  e1 := AET;
  // * === SCANBEAM BOUNDARY PROCESSING ================================ */

  // * Set up bundle fields of first edge */
  AET.bundle[ABOVE][AET.typ] := Ord(AET.top.y <> Yb);
  AET.bundle[ABOVE][Ord(AET.typ = 0)] := FFALSE;
  AET.bstate[ABOVE] := UNBUNDLED;

  next_edge := AET.next;
  while next_edge <> nil do
  begin
    // * Set up bundle fields of next edge */
    next_edge.bundle[ABOVE][next_edge.typ] := Ord(next_edge.top.y <> Yb);
    next_edge.bundle[ABOVE][Ord(next_edge.typ = 0)] := FFALSE;
    next_edge.bstate[ABOVE] := UNBUNDLED;

    // * Bundle edges above the scanbeam boundary if they coincide */
    // EQ may  be dropped???
    if (next_edge.bundle[ABOVE][next_edge.typ] <> 0) then
    begin
      if (EQ(e0.xb, next_edge.xb) and EQ(e0.dx, next_edge.dx) and
        (e0.top.y <> Yb)) then
      begin
        next_edge.bundle[ABOVE][next_edge.typ] := next_edge.bundle[ABOVE]
          [next_edge.typ] xor e0.bundle[ABOVE][next_edge.typ];
        next_edge.bundle[ABOVE][Ord(next_edge.typ = 0)] :=
          e0.bundle[ABOVE][Ord(next_edge.typ = 0)];
        next_edge.bstate[ABOVE] := BUNDLE_HEAD;
        e0.bundle[ABOVE][gpc.clip] := FFALSE;
        e0.bundle[ABOVE][gpc.subj] := FFALSE;
        e0.bstate[ABOVE] := BUNDLE_TAIL;
      end;
      e0 := next_edge;
    end;
    next_edge := next_edge.next;
  end;
end;

// ==========================================
procedure AddChainEdgesAt(Yb: double; var local_min: Plmt_node;
  var Loc_min_idx: integer);
var
  edge: Pedge_node;
begin
  { If LMT node corresponding to yb exists }
  if local_min <> nil then
  begin
    while (local_min.y = Yb) do
    begin
      { Add edges starting at this local minimum (==yb)to the AET }
      edge := local_min.Base_edge;
      add_edge_to_aet(AET, edge, nil); // they are inserted in x,dx order.!
      if (edge <> nil) then
        UpdateScanLineQ(edge.top.y);
      // top of edge becomes a candidate for a scan line
      if (Loc_min_idx = lm_List.Count) then
        break;
      local_min := lm_List[Loc_min_idx];
      Inc(Loc_min_idx);
    end;
    if (Loc_min_idx < lm_List.Count) then
      UpdateScanLineQ(local_min.y);
  end;
  // ===============================================
end;

// =========================================================
procedure InitTristrip(op: Tgpc_op; subj: Pgpc_polygon; clip: Pgpc_polygon;
  result: Pgpc_tristrip);
var // NOT Used
  lmt, local_min: Plmt_node;
  Loc_min_idx: integer;

  BaseLineY: double; // First scan line ...
  TerminalLineY: double;

begin
  AET := nil;
  clipop := op;
  c_heap := nil;
  s_heap := nil;
  lmt := nil;
  // AssignFile(VattiMessages,'TRimesag.txt');
  // ReWrite(VattiMessages);

  // * Test for trivial NULL result cases */
  if (((subj.num_contours = 0) and (clip.num_contours = 0)) or
    ((subj.num_contours = 0) and ((op = GPC_INT) or (op = GPC_DIFF))) or
    ((clip.num_contours = 0) and (op = GPC_INT))) then
  begin
    result.num_strips := 0;
    result.strip := nil;
    exit;
  end;

  // * Identify potentialy contributing contours */
  if (((op = GPC_INT) or (op = GPC_DIFF)) and (subj.num_contours > 0) and
    (clip.num_contours > 0)) then
  begin
    minimax_test(subj, clip, op);
  end;

  // * Build LMT */

  // * Return a NULL result if no contours contribute */
  { Build LMT }
  sbt_Minimum := DBL_MAX;
  sbt_Maximum := -DBL_MAX;

  lm_List := TList.Create();
  lm_List.Clear;
  itList := TList.Create();
  itList.Clear;
  Lmt_NextFreeIndex := 0;

  Loc_min_idx := 0;

  LmtList_NextFreeIndex := 0;
  // * Build Local Minima Tables to start  */

  if (subj.num_contours > 0) then
    s_heap := build_lmt(lmt, subj, gpc.subj, op);
  if (clip.num_contours > 0) then
    c_heap := build_lmt(lmt, clip, gpc.clip, op);

  lm_List.Sort(lmCompare);
  { Return a NULL result if no contours contribute }
  // if lmt = nil then
  if LmtList_NextFreeIndex = 0 then
  // if lmt = nil then
  begin
    result.num_strips := 0;
    result.strip := nil;
    // reset_lmt(lmt);
    EndCleanUp();
    exit;
  end;

  UpdateScanLineQ(DBL_MAX);

  TriStripList := nil;
  TriStripListLast := nil; // used by triangulation procdures globally

  Loc_min_idx := 0;
  // list out the lmt....
  local_min := lm_List[0];
  Inc(Loc_min_idx);
  BaseLineY := local_min.y;
  UpdateScanLineQ(BaseLineY); // 1st scan line
  PopScanLineQ(Yt);

end;

procedure MPC_ClipTriChains(op: Tgpc_op; subj: Pgpc_polygon;
  clip: Pgpc_polygon);

// procedure gpc_tristrip_clip(op: Tgpc_op; subj: Pgpc_polygon; clip: Pgpc_polygon;
// result: Pgpc_tristrip);
var
  edge: Pedge_node;
  lmt, local_min: Plmt_node;
  Loc_min_idx: integer;

  BaseLineY: double; // First scan line ...
  TerminalLineY: double;
  result: Pgpc_tristrip;

begin

  InitTristrip(op, subj, clip, result);

  AET := nil;
  clipop := op;
  c_heap := nil;
  s_heap := nil;
  lmt := nil;


  // AssignFile(VattiMessages,'TRimesag.txt');
  // ReWrite(VattiMessages);

  // * Test for trivial NULL result cases */
  if (((subj.num_contours = 0) and (clip.num_contours = 0)) or
    ((subj.num_contours = 0) and ((op = GPC_INT) or (op = GPC_DIFF))) or
    ((clip.num_contours = 0) and (op = GPC_INT))) then
  begin
    result.num_strips := 0;
    result.strip := nil;
    exit;
  end;

  // * Identify potentialy contributing contours */
  if (((op = GPC_INT) or (op = GPC_DIFF)) and (subj.num_contours > 0) and
    (clip.num_contours > 0)) then
  begin
    minimax_test(subj, clip, op);
  end;

  // * Build LMT */

  // * Return a NULL result if no contours contribute */
  { Build LMT }
  sbt_Minimum := DBL_MAX;
  sbt_Maximum := -DBL_MAX;

  lm_List := TList.Create();
  lm_List.Clear;
  itList := TList.Create();
  itList.Clear;
  Lmt_NextFreeIndex := 0;
  Loc_min_idx := 0;
  LmtList_NextFreeIndex := 0;
  if (subj.num_contours > 0) then
    s_heap := build_lmt(lmt, subj, gpc.subj, op);
  if (clip.num_contours > 0) then
    c_heap := build_lmt(lmt, clip, gpc.clip, op);

  lm_List.Sort(lmCompare);
  { Return a NULL result if no contours contribute }
  // if lmt = nil then
  if LmtList_NextFreeIndex = 0 then
  // if lmt = nil then
  begin
    result.num_strips := 0;
    result.strip := nil;
    // reset_lmt(lmt);
    EndCleanUp();
    exit;
  end;

  // * Build scanbeam table from scanbeam tree */
  // MALLOC(pointer(sbt), sbt_entries * sizeof(double), 'sbt creation');
  // build_sbt(scanbeam, sbt, sbtree);

  UpdateScanLineQ(DBL_MAX);

  TriStripList := nil;
  TriStripListLast := nil; // used by triangulation procdures globally

  Loc_min_idx := 0;
  // list out the lmt....
  local_min := lm_List[0];
  Inc(Loc_min_idx);
  BaseLineY := local_min.y;
  UpdateScanLineQ(BaseLineY); // 1st scan line
  PopScanLineQ(Yt);
  // * Process each scanbeam */
  // while (scanbeam < sbt_entries) do
  while (Yt <= sbt_Maximum) do
  begin
    Yb := Yt;
    AddChainEdgesAt(Yb, local_min, Loc_min_idx);
    if (Yt < sbt_Maximum) then
    begin
      PopScanLineQ(Yt);
    end;
    // * Create bundles within AET */
    FormAETBundles();
    ProcessEdgesSpanningDividingLine(false);

    // * Delete terminating edges from the AET, otherwise compute xt */
    DeleteTerminatingEdgesFromAET();

    if (Yb <> Yt) then
    begin
      // ===========================
      { Process each node in the intersection table }
      ProcessWithinBeamIntersections(false);
      // =====================================
      { Prepare for next scanbeam }
      PrepareForNextScanBeam();
      // Horizontal edge sequences are tackled in forward direction!.
    end; // if (yb <> yt)

    if (Yb = Yt) then
      break;
  end; { === END OF SCANBEAM PROCESSING ================================== }

  FreeScanLineQ();
  // The result is  available as TriStripList
  // ================================================
  // * Generate result tristrip from tlist */
  // copy this wonderful piece of code to the c++ MPC clipper....
  // result.strip := nil;
  // result.num_strips := count_tristrips(TriStripList);
  // if (result.num_strips > 0) then
  // GenerateTriStrips(result);

  EndCleanUp();
  // closeFile(VattiMessages);
end;

procedure GenerateContours(var result: Pgpc_polygon);
// Generate result contours from TriStripList,linked list of contour doublet chains
var

  s: integer;
  tn, tnn: Ppolygon_node;
  lt, ltn, rt, rtn: Pvertex_node;
  i,j, iTop: integer;
  Tvertex: Pgpc_vertex_array;
  NumofVertices, cindex: integer;
  RtVertices: integer;
begin
//Use only those  contours with active count >2
  result.num_contours := count_tristrips(TriStripList);
if (result.num_contours>0) then
begin
  MALLOC(pointer(result.contour), result.num_contours *
    sizeof(Tgpc_vertex_list), 'contour creation');
  MALLOC(pointer(result.hole), result.num_contours *
      sizeof(integer), 'hole flag table creation');
  // result.contour:=new gpc_vertex_list[ result.num_contours ];
  s := 0;

  tnn := TriStripList;
  while (tnn <> nil) do
  begin
    tn := tnn;
    tnn := tn.next; // assert valid contour ??? how ??
   //use only Non trivial contours
    if (tn.active > 2) then
    begin
     NumofVertices := tn.active;
     result.contour[s].num_vertices := NumofVertices;
     result.hole[s]:=FFALSE ; //No holes anyway but be explicit

      MALLOC(pointer(result.contour[s].vertex), NumofVertices * sizeof(Tgpc_vertex),
        'vertex  creation');

      Tvertex := result.contour[s].vertex;

      lt := tn.v[LEFT]; // use a better notation
      rt := tn.v[RIGHT];

    // copy the strip points to contour[s]
     i := 0;   NumofVertices:=0;
     while (lt <> nil) do
      begin
        ltn := lt.next;
        Tvertex[i].x := lt.x;
        Tvertex[i].y := lt.y;
    //    Inc(NumofVertices);
        Inc(i);
        lt := ltn;
      end;
      // copy rt in reverse order
      RtVertices := 0;
      rtn := rt;

      while (rtn <> nil) do
      begin
        Inc(RtVertices);
        rtn := rtn.next;
      end;
      iTop := i + RtVertices - 1;
      j := iTop;
      while (rt <> nil) do
      begin
        rtn := rt.next;
        Tvertex[j].x := rt.x;
        Tvertex[j].y := rt.y;
       // Inc(NumofVertices);
        Dec(j);
        rt := rtn;
      end;
     Inc(s);
    end; // tn.active > 2
   end // while tn !=nil
 end; // num_contours > 0

 // if (result.num_contours > s) then
  //  result.num_contours := s;

end; // ---------------------TriContour-----------------------

procedure gpc_tristrip_clip(op: Tgpc_op; subj: Pgpc_polygon; clip: Pgpc_polygon;
  Tresult: Pgpc_tristrip);
var
  edge: Pedge_node;
  // global TriStripList :Ppolygon_node;
begin
  // edit chains to return strips..
  // use the 2 branches of the Murta Clipper  to get a strip of triangles
  MPC_ClipTriChains(op, subj, clip);

  Tresult.strip := nil;
  Tresult.num_strips := count_tristrips(TriStripList);
  if (Tresult.num_strips > 0) then
    GenerateTriStrips(Tresult);
 // ReleaseTriStrips();
end;

procedure gpc_tri_poly(op: Tgpc_op; subj: Pgpc_polygon; clip: Pgpc_polygon;
  presult: Pgpc_polygon; OLDrun:boolean);
var
  edge: Pedge_node;
  ContourCount:integer;
  // global TriStripList :Ppolygon_node;
begin
  // use the 2 branches of the Murta Clipper  to get a collection
  // of single minima contours ..A La Vatti...
  if( NOT OLDrun)then
   MPC_ClipTriChains(op, subj, clip); // modify later ...

 ContourCount := count_tristrips(TriStripList);

  if (ContourCount > 0) then
  GenerateContours(Presult);
  ReleaseTriStrips();
 end;

// ==::WARNING====Area routines yet to be checked....
procedure CArea(Gcontour: Tgpc_vertex_list; var x: double);
var
  gvtx: Pgpc_vertex_array;
  Contour_Area: double;
  LastIndex: integer;
  vidx: integer;
begin
  // Murta does Not provide consistently oriented contours..
  gvtx := Gcontour.vertex;
  LastIndex := Gcontour.num_vertices - 1;
  if (LastIndex < 2) then
    Contour_Area := 0.0
  else
  begin
    Contour_Area := (gvtx[0].x + gvtx[LastIndex].x) *
      (gvtx[0].y - gvtx[LastIndex].y);

    for vidx := 0 to LastIndex - 1 do
      Contour_Area := Contour_Area + (gvtx[vidx + 1].x + gvtx[vidx].x) *
        (gvtx[vidx + 1].y - gvtx[vidx].y);
  end;
  if (Contour_Area < 0) then
    Contour_Area := -Contour_Area;
  x := Contour_Area / 2.0;
end;

procedure PArea(Gpoly: Pgpc_polygon; var x: double);
var
  gvtx: Pgpc_vertex_array;
  poly_Area: double;
  Contour_Area: double;
  LastIndex, ci: integer;
  xc: double;
begin
  poly_Area := 0.0;
  x := 0.0;
  for ci := 0 to Gpoly.num_contours - 1 do
  begin
    CArea(Gpoly.contour[ci], Contour_Area);
    // Only For holes,contour area should be negative
    // assume holes are identified correctly ...
    if (Gpoly.hole[ci] = 1) then
    begin
      if (Contour_Area > 0) then
        Contour_Area := -Contour_Area;
    end
    else
    begin
      if (Contour_Area < 0) then
        Contour_Area := -Contour_Area;
    end;
    poly_Area := poly_Area + Contour_Area;
  end;
  x := poly_Area;
end;

procedure TriStripArea(tristrip: Pgpc_tristrip; var StripArea: double);
var

  V3: Tgpc_vertex;
  V2: Tgpc_vertex;
  V1: Tgpc_vertex;
  VertexIndex: integer;
  strip: Tgpc_vertex_list;
  StripIndex: integer;
  x: double;
  triarea: double;
begin
  // use a better routine  like polyarea....
  // In the strip with NOT identical arm lengths ....degenerate
  // triangles do appear ..are ==0.0

  x := 0.0;
  for StripIndex := 0 to tristrip.num_strips - 1 do
  begin
    strip := tristrip.strip[StripIndex];

    for VertexIndex := 0 to strip.num_vertices - 3 do
    begin

      begin
        V1 := strip.vertex[VertexIndex];
        V2 := strip.vertex[VertexIndex + 1];
        V3 := strip.vertex[VertexIndex + 2];
      end;
      triarea := V1.x * (V2.y - V3.y) + V2.x * (V3.y - V1.y) + V3.x *
        (V1.y - V2.y);
      if (triarea < 0.0) then
        triarea := -triarea;

      x := x + triarea;

    end;
  end;
  StripArea := x / 2.0;
end;

// ======================================
// End of file: gpc.pas
// ======================================

end.
