
__kernel void hello_kernel(	__global const float * xVector,
							__global const float * yVector,
							__global Delaunay::Vertex_handle * theVertexHandle )
{
	int gid = get_global_id(0);
	Pt2 xy = Pt2(xVector[gid], yVector[gid]);
	theVertexHandle[gid] = tr.insert(xy);		
}

