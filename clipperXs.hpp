

class Clipper : public virtual ClipperBase
{
public:
 ....
 ....
 ....
int m_Trivials;
bool FixUpFailureReported;
char XMethod;   //bubble/mergesort/Insertionsort =='B','M','I'
//========================================
 .
.
.
 bool ProcessIntersections(const cInt topY);
 
 void InsertsecondBeforefirstInSel(TEdge & first, TEdge& second);
 void BSBuildIntersectList(const cInt topY);  //Bubble..Angus Special
 void MSBuildIntersectList(const cInt topY);  //Merge from clipper 2..Angus Clipper 2
 void ISBuildIntersectList(const cInt topY);  //Vatti fromp Mpc ..Murtas Vatti
 void ProcessIntersectList();
 .
 .
 .
};
//------------------------------------------------------------------------------



