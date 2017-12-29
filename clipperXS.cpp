
struct TEdge {
  IntPoint Bot;
  IntPoint Curr;
  IntPoint Top;
  IntPoint Delta;
  double Dx;
  PolyType PolyTyp;
  EdgeSide Side;
  int WindDelta; //1 or -1 depending on winding direction
  int WindCnt;
  int WindCnt2; //winding count of the opposite polytype
  int OutIdx;
  TEdge *Next;
  TEdge *Prev;
  TEdge *NextInLML;
  TEdge *NextInAEL;
  TEdge *PrevInAEL;
  TEdge *NextInSEL;
  TEdge *PrevInSEL;
  TEdge *merge_jump;

};

//------------------------------------------------------------------------------



bool Clipper::ProcessIntersections(const cInt topY)
{
  if( !m_ActiveEdges )
  return true;


  try {
  if(XMethod=='B')
	 BSBuildIntersectList(topY);
  else  if(XMethod=='I' )
	 ISBuildIntersectList(topY);
 else if(XMethod='M')
   MSBuildIntersectList(topY);
  else
	ISBuildIntersectList(topY);

	size_t IlSize = m_IntersectList.size();
  if (IlSize == 0) return true;
    m_OpInterSectionsCount+=IlSize;
    if (IlSize == 1 || FixupIntersectionOrder())
		 ProcessIntersectList();
	else { m_WithinBeam=false;
	return false;
	}
  }
  catch(...) 
  {
    m_SortedEdges = 0;
    DisposeIntersectNodes();
	throw clipperException("ProcessIntersections error");
  }
  m_SortedEdges = 0;

  return true;
}
//------------------------------------------------------------------------------


  //------------------------------------------------------------------------------


//============================new code ======================
  inline void Clipper::CopyActivesToSELAdjustCurrX(const cInt topY)
  {
	TEdge* e = m_ActiveEdges;
    while (e) {
	  e->PrevInSEL = e->PrevInAEL;
	  e->NextInSEL = e->NextInAEL;
	  e->Curr.X = TopX(*e, topY);
	  e = e->NextInAEL;
    }
  }

  void Clipper::InsertNewIntersectNode(Active &e1, Active &e2, cInt topY)
  {
	IntPoint pt; 
	 IntersectPoint(&e1, &e2, pt);
	//Rounding errors can occasionally place the calculated intersection
	//point either below or above the scanbeam, so check and correct ...
	if (pt.Y > e1.Curr.Y) {
	  pt.Y = e1.Curr.Y;      //e1.curr.y is still the bottom of scanbeam
							 //use the more vertical of the 2 edges to derive pt.X ...
	  if (Abs(e1.Dx) < Abs(e2.Dx)) pt.X = TopX(e1, pt.Y);
	  else pt.X = TopX(e2, pt.Y);
	}
	else if (pt.Y < topY) {
	  pt.Y = topY;          //top_y is at the top of the scanbeam

	  if (e1.Top.Y == topY) pt.X = e1.Top.X;
	  else if (e2.Top.Y == topY) pt.X = e2.Top.X;
	  else if (Abs(e1.Dx) < Abs(e2.Dx)) pt.X = e1.Curr.X;
	  else pt.X = e2.Curr.X;
	}

	IntersectNode *node = new IntersectNode();
	node->Edge1 = &e1;
	node->Edge2 = &e2;
	node->Pt = pt;
	m_IntersectList.push_back(node);
  }
  //=======================================================
   void Clipper::ISBuildIntersectList(const  cInt topY)
   { 
   TEdge *SEL,*pcSEL ,*CSEL;
	  TEdge *es;
	TEdge* e = m_ActiveEdges;

	while (e) {
	  e->PrevInSEL = e->PrevInAEL;
	  e->NextInSEL = e->NextInAEL;
	  e->Curr.X = TopX(*e, topY);    //xt on top scan line
	  e = e->NextInAEL;
	}

	SEL=NULL;
	 /*   Vatti Recipe++ as used in gpc by Murta(?), */
	for (es= m_ActiveEdges; es; es= es->NextInAEL)
	  SELInsert(&SEL,  es,topY);


   
	   //=====================Yt=topY; Yb=???
   }

  void Clipper::SELInsert(TEdge **pSEL, TEdge *edge,const  cInt topY)
{  /* de tailed  ,cut pasted..mutilated from MPC code*/
/* cleaned   9 oct 2012 */

  TEdge *SEL;
  TEdge *AddSELNode, *BackSELNode, *TestSELNode;

  /*  de tailed code  18-jan-2011  uses do while  */
	 if(!edge)return;

	 AddSELNode= edge;
	 AddSELNode->PrevInSEL= NULL;
	 AddSELNode->NextInSEL=NULL;

	 SEL=*pSEL;    /* Note that this SEL is local to  this routine */
	 if(!SEL)
	  {	 *pSEL=AddSELNode; //1st node in SEL
		  return;
	  };
	 /* Compute intersection between  edge and ST edges */


  //--------------a pre  scan of AEL is required to get the xts..?????
	int lCnt = 0;   //count # of intersections

	  TestSELNode=SEL; BackSELNode=NULL;
	  while(TestSELNode && (edge->Curr.X < TestSELNode->Curr.X))
	   {  InsertNewIntersectNode(*TestSELNode, *edge, topY);
		  lCnt++;
		 BackSELNode=TestSELNode;
		 TestSELNode=TestSELNode->PrevInSEL;
		}
	 /* should exit  from here when No more intersections .. */

	if (!BackSELNode)     //before first node
	{  AddSELNode->PrevInSEL=SEL;
	   SEL=AddSELNode;
	}
	else
	BackSELNode->PrevInSEL=AddSELNode;    //insert in linked list

	if(TestSELNode)              //if not last node
	   AddSELNode->PrevInSEL= TestSELNode;
	*pSEL=SEL;

  } /*SELInsert */


 /*      ---------------------------------------   */
  //------------------------------------------------------------------------------
  void Clipper::InsertsecondBeforefirstInSel(TEdge & first, TEdge& second)
  { //remove second from list ...
	TEdge *prev = second.PrevInSEL;
	TEdge *next = second.NextInSEL;
	prev->NextInSEL = next; //always a prev since we're moving from right to left
	if (next) next->PrevInSEL = prev;
	//insert back into list ...
	prev = first.PrevInSEL;
	if (prev) prev->NextInSEL = &second;
	first.PrevInSEL = &second;
	second.PrevInSEL = prev;
	second.NextInSEL = &first;
  }
  void Clipper::MSBuildIntersectList(const  cInt topY)
  {	TEdge *e = m_ActiveEdges;
  
	if (!e || !e->NextInAEL) return;

	CopyActivesToSELAdjustCurrX(topY);
	m_SortedEdges = e;   IntPoint pt;

	//Merge sort actives into their new positions at the top of scanbeam, and
    //create an intersection node every time an edge crosses over another ...
    //see also https://stackoverflow.com/a/46319131/359538

	TStart=StopTimer()*10000;
	int mul = 1;
    while (true) {

      TEdge *first = m_SortedEdges, *second = NULL, *baseE, *prev_base = NULL, *tmp;
      //sort successive larger 'mul' count of nodes ...
      while (first) {
        if (mul == 1) {
		  second = first->NextInSEL;
          if (!second) break;
		  first->merge_jump = second->NextInSEL;
        }
        else {
          second = first->merge_jump;
          if (!second) break;
          first->merge_jump = second->merge_jump;
        }

        //now sort first and second groups ...
        baseE = first;
		int lCnt = mul, rCnt = mul;
		while (lCnt > 0 && rCnt > 0) {
		  if (second->Curr.X < first->Curr.X) {
		 // create one or more Intersect nodes ///////////
			tmp = second->PrevInSEL;
		   for (int i = 0; i < lCnt; ++i) {
			 InsertNewIntersectNode(*tmp, *second, topY);
			  tmp = tmp->PrevInSEL;
			}
			/////////////////////////////////////////////////

            if (first == baseE) {
              if (prev_base) prev_base->merge_jump = second;
              baseE = second;
              baseE->merge_jump = first->merge_jump;
			  if (!first->PrevInSEL) 	m_SortedEdges = second;
            }
			tmp = second->NextInSEL;
			//now move the out of place edge to it's new position in SEL ...
			//============================================

		   InsertsecondBeforefirstInSel(*first, *second);
            second = tmp;
            if (!second) break;
            --rCnt;
          }
          else {
			first = first->NextInSEL;
            --lCnt;
          }
        }
        first = baseE->merge_jump;
        prev_base = baseE;
      }
	  if (!m_SortedEdges->merge_jump) break;
      else mul <<= 1;
	}
	
   m_SortedEdges = NULL;
   //important ,as other routine which follow later expect it ==0
  }
  //------------------------------------------------------------------------------

 
  //------------------------------------------------------------------------------
void Clipper::BSBuildIntersectList(const cInt topY)
{
  if ( !m_ActiveEdges ) return;
 
  //prepare for sorting ...
  TEdge* e = m_ActiveEdges;
 
  m_SortedEdges = e;
  while( e )
  {
	e->PrevInSEL = e->PrevInAEL;
	e->NextInSEL = e->NextInAEL;
	e->Curr.X = TopX( *e, topY );
	e = e->NextInAEL;
  }
  //=======================
  //bubblesort ...
  bool isModified;
  do
  {
    isModified = false;
    e = m_SortedEdges;
	while( e->NextInSEL )
    {
      TEdge *eNext = e->NextInSEL;
	  IntPoint Pt;
      if(e->Curr.X > eNext->Curr.X)
      {
		IntersectPoint(e, eNext, Pt);
        IntersectNode * newNode = new IntersectNode;
        newNode->Edge1 = e;
		newNode->Edge2 = eNext;
        newNode->Pt = Pt;
		m_IntersectList.push_back(newNode);

        SwapPositionsInSEL(e, eNext);
        isModified = true;
	  }
      else
        e = eNext;
	}
    if( e->PrevInSEL ) e->PrevInSEL->NextInSEL = 0;
	else break;
  }
  while ( isModified );
	
  m_SortedEdges = 0;
   //important ,as other routine which follow later expect it ==0 .

}
//------------------------------------------------------------------------------

} //ClipperLib namespace
