using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Autodesk.Max;
using SharpDX;
using System.Reflection;
using ROD_core;
using ROD_core.Graphics.Assets;
using System.Runtime.InteropServices;

namespace RODExporter
{

    public class RODExportG
    {
        public IGlobal maxGlobal;
        public IInterface13 maxInterface;

        public static List<uint> SelectedNodes = new List<uint>();
        public static Semantic semantic;

        public static void SetSemantic(Semantic _semantic)
        {
            semantic = _semantic;
        }
        public static void SetSemantic(string _semantic_string)
        {
            List<string> semantic_Keywords = _semantic_string.Split(' ').ToList();
            List<Semantic> list_keywords = new List<Semantic>();
            foreach (string keyword in semantic_Keywords)
            {
                list_keywords.Add((Semantic)Enum.Parse(typeof(Semantic), keyword));
            }
            semantic = list_keywords[0];
            for (int i = 1; i < list_keywords.Count; i++)
            {
                semantic |= list_keywords[i];
            }
        }

        public static void SelectNode(uint _handle)
        {
            SelectedNodes.Add(_handle);
        }

        public static bool Export_To(string _filepath)
        {
            RODExportG r = new RODExportG();
            foreach (uint _handle in SelectedNodes)
            {
                IINode _node = r.maxInterface.GetINodeByHandle(_handle);
                IObject _gObject = _node.ObjectRef;
                ITriObject _mObject = (ITriObject)_gObject;
                IMesh _mMesh = _mObject.Mesh;
                _mMesh.BuildNormals();
                ComputeVertexData(_mMesh, semantic, _filepath);

            }
            return true;
        }
        public static bool Bone_To(int _frame)
        {
            RODExportG r = new RODExportG();
            int _ticks_per_frame = r.maxGlobal.TicksPerFrame;
            List<IQuat> _rotations = new List<IQuat>();
            foreach (uint _handle in SelectedNodes)
            {
                IINode _node = r.maxInterface.GetINodeByHandle(_handle);
                IInterval interval=r.maxGlobal.Interval.Create();
                interval.SetInfinite();
                IMatrix3 _node_matrix = _node.GetNodeTM(_frame*_ticks_per_frame, interval);
                IMatrix3 _parent_matrix = _node.GetParentTM(_frame * _ticks_per_frame);
                _parent_matrix.Invert();
                IMatrix3 _local_matrix = _node_matrix.Multiply(_parent_matrix);
                IQuat _local_Rotation=r.maxGlobal.IdentQuat;
                _local_Rotation.Set(_local_matrix);
                IEulerAnglesValue euler = r.maxGlobal.EulerAnglesValue.Create(_local_Rotation);
                float[] temp = new float[3];
                for (int i = 0; i < euler.Angles.Length; i++ )
                {
                    float a = ROD_core.Mathematics.Math_helpers.ToDegrees(euler.Angles[i]);
                    temp[i] = a;
                }
                Vector3 lisible = new Vector3(temp);
                _rotations.Add(_local_Rotation);
            }
            return true;
        }
        static IntPtr IntPtrFromFloat(float f)
        {
            unsafe
            {
                return (*(IntPtr*)&f);
            }
        }

        public RODExportG()
        {
            maxGlobal = Autodesk.Max.GlobalInterface.Instance;
            maxInterface = maxGlobal.COREInterface13;
        }

        public static void ComputeVertexData(IMesh _mMesh, Semantic _semantic, string _filepath)
        {
            Dictionary<int, VertexUndivided> verticesFullData = new Dictionary<int, VertexUndivided>();
            List<FaceData> facesFullData = new List<FaceData>();
            List<List<VertexDivided>> verticesParsedData = new List<List<VertexDivided>>();

            IList<IFace> faces = _mMesh.Faces;
            IList<ITVFace> Tfaces = _mMesh.TvFace;
            IList<IPoint3> vertices = _mMesh.Verts;
            IList<IPoint3> Tvertices = _mMesh.TVerts;
            foreach (IPoint3 _v in vertices)
            {
                float temp = _v.Y;
                _v.Y = _v.Z;
                _v.Z = temp;
            }

            for (int _fID = 0; _fID < faces.Count; _fID++)
            {
                FaceData _face = new FaceData((int)faces[_fID].SmGroup);

                // vectors are inverted to make up for max being clockwise
                Vector3 A_B = vertices[(int)faces[_fID].GetVert(0)].convertToVector3() - vertices[(int)faces[_fID].GetVert(1)].convertToVector3();
                Vector3 A_C = vertices[(int)faces[_fID].GetVert(0)].convertToVector3() - vertices[(int)faces[_fID].GetVert(2)].convertToVector3();
                Vector3 U = Tvertices[(int)Tfaces[_fID].GetTVert(0)].convertToVector3() - Tvertices[(int)Tfaces[_fID].GetTVert(1)].convertToVector3();
                Vector3 V = Tvertices[(int)Tfaces[_fID].GetTVert(0)].convertToVector3() - Tvertices[(int)Tfaces[_fID].GetTVert(2)].convertToVector3();

                Vector3 normUnsure = Vector3.Cross(A_B, A_C);
                normUnsure.Normalize();

                float area = U.X * V.Y - U.Y * V.X;
                int sign = area < 0 ? -1 : 1;
                Vector3 tangent = new Vector3(0, 0, 1);
                tangent.X = A_B.X * V.Y - U.Y * A_C.X;
                tangent.Y = A_B.Y * V.Y - U.Y * A_C.Y;
                tangent.Z = A_B.Z * V.Y - U.Y * A_C.Z;
                tangent.Normalize();
                tangent *= sign;

                for (int i = 0; i < 3; i++)
                {
                    _face.vertices.Add((int)faces[_fID].GetVert(i));


                    if (verticesFullData.ContainsKey((int)faces[_fID].GetVert(i)))
                    {
                        VertexUndivided _v = verticesFullData[(int)faces[_fID].GetVert(i)];
                        _v.faceInfo.Add(new PerFaceInfo(_fID, (int)faces[_fID].SmGroup, (Tvertices[(int)Tfaces[_fID].GetTVert(i)]).convertToVector2(), normUnsure, tangent));
                    }
                    else
                    {
                        VertexUndivided _v = new VertexUndivided();
                        _v.ID = faces[_fID].GetVert(i);
                        _v.pos = (vertices[(int)_v.ID]).convertToVector3();
                        _v.faceInfo.Add(new PerFaceInfo(_fID, (int)faces[_fID].SmGroup, (Tvertices[(int)Tfaces[_fID].GetTVert(i)]).convertToVector2(), normUnsure, tangent));
                        verticesFullData.Add((int)faces[_fID].GetVert(i), _v);
                    }
                }

                facesFullData.Add(_face);
            }
            List<int> IndexBuffer = new List<int>();
            List<VertexDivided> VertexBuffer = new List<VertexDivided>();
            // vertex index in vertexfullData "undivided" et sa valeur en fonction du SMG
            Dictionary<int, Dictionary<int, int>> VertexDictionary = new Dictionary<int, Dictionary<int, int>>();

            Mesh mesh = new Mesh();


            for (int _faceID = 0; _faceID < facesFullData.Count; _faceID++)
            {
                foreach (int _vertex in facesFullData[_faceID].vertices)
                {
                    Dictionary<int, int> vertexTranslation;
                    int _vID = (int)verticesFullData[_vertex].ID;
                    List<PerFaceInfo> unitedVertex = verticesFullData[_vertex].faceInfo.Where(x => x.SMG == facesFullData[_faceID].SMG).ToList();
                    if (!VertexDictionary.ContainsKey(_vID))
                    {
                        VertexDictionary.Add(_vID, new Dictionary<int, int>());
                    }
                    vertexTranslation = VertexDictionary[_vID];
                    /*
                    if (vertexTranslation.ContainsKey(facesFullData[_faceID].SMG))
                    {
                        IndexBuffer.Add(vertexTranslation[facesFullData[_faceID].SMG]);
                    }
                    else
                    {
                     */
                        VertexDivided _newVertex = new VertexDivided();
                        _newVertex.pos = verticesFullData[_vertex].pos;
                        
                        _newVertex.UV.X = verticesFullData[_vertex].faceInfo.Where(x => x.ID == _faceID).FirstOrDefault().UV.X;
                        _newVertex.UV.Y = 1-verticesFullData[_vertex].faceInfo.Where(x => x.ID == _faceID).FirstOrDefault().UV.Y;
                        Vector3 _normal_aggreagate = new Vector3(0, 0, 0);
                        Vector3 _tangent_aggreagate = new Vector3(0, 0, 0);
                        foreach (PerFaceInfo _FI in unitedVertex)
                        {
                            _normal_aggreagate += _FI.normal;
                            _tangent_aggreagate += _FI.tangent;
                        }
                        _normal_aggreagate.Normalize();
                        _tangent_aggreagate.Normalize();
                        _newVertex.normal = _normal_aggreagate;
                        _newVertex.tangent = _tangent_aggreagate;
                        _newVertex.binormal = Vector3.Cross(_normal_aggreagate, _tangent_aggreagate);
                        //vertexTranslation.Add(facesFullData[_faceID].SMG, VertexBuffer.Count);
                        IndexBuffer.Add(VertexBuffer.Count);
                        VertexBuffer.Add(_newVertex);
                    //}
                }
            }
            mesh._indexStream = new IndexStream(IndexBuffer.Count, typeof(UInt16), true, true);
            mesh._vertexStream = new VertexStream(VertexBuffer.Count, true, true, _semantic);
            foreach (int id in IndexBuffer)
            {
                
                UInt16 _id = Convert.ToUInt16(id);
                VertexDivided res = VertexBuffer[_id];
                mesh._indexStream.WriteIndex(_id);
            }
            foreach (VertexDivided vd in VertexBuffer)
            {
                if(mesh._boundingBox.Minimum == null)
                {
                    mesh._boundingBox.Minimum = new Vector3(vd.pos.X, vd.pos.Y, vd.pos.Z);
                    mesh._boundingBox.Maximum = new Vector3(vd.pos.X, vd.pos.Y, vd.pos.Z);
                }
                mesh._boundingBox.Minimum.X = Math.Min(mesh._boundingBox.Minimum.X, vd.pos.X);
                mesh._boundingBox.Minimum.Y = Math.Min(mesh._boundingBox.Minimum.Y, vd.pos.Y);
                mesh._boundingBox.Minimum.Z = Math.Min(mesh._boundingBox.Minimum.Z, vd.pos.Z);
                mesh._boundingBox.Maximum.X = Math.Max(mesh._boundingBox.Maximum.X, vd.pos.X);
                mesh._boundingBox.Maximum.Y = Math.Max(mesh._boundingBox.Maximum.Y, vd.pos.Y);
                mesh._boundingBox.Maximum.Z = Math.Max(mesh._boundingBox.Maximum.Z, vd.pos.Z);
                object[] obj = new object[] { vd.pos, vd.normal, vd.UV, vd.binormal, vd.tangent };
                //object[] obj = new object[] { vd.pos, vd.normal, vd.UV};
                mesh._vertexStream.WriteVertex(obj);
            }
            Mesh.saveToFile(mesh, _filepath);
        }



    }
    public struct PerFaceInfo
    {
        public int ID;
        public int SMG;
        public Vector2 UV;
        public Vector3 normal;
        public Vector3 tangent;

        public PerFaceInfo(int ID, int SMG, Vector2 UV, Vector3 normal, Vector3 tangent)
        {
            this.ID = ID;
            this.SMG = SMG;
            this.UV = UV;
            this.normal = normal;
            this.tangent = tangent;
        }
    }

    public class FaceData
    {
        public int SMG;
        public List<int> vertices;

        public FaceData(int SMG)
        {
            this.SMG = SMG;
            vertices = new List<int>();
        }
    }

    public class VertexUndivided
    {
        public uint ID;
        public Vector3 pos;
        public List<PerFaceInfo> faceInfo;

        public VertexUndivided()
        {
            faceInfo = new List<PerFaceInfo>();
        }
    }
    public struct VertexDivided
    {
        public Vector3 pos;
        public Vector3 normal;
        public Vector2 UV;
        public Vector3 binormal;
        public Vector3 tangent;

    }


    

    public static class maxHelper
    {
        public static Vector3 convertToVector3(this IPoint3 _input)
        {
            Vector3 _output = new Vector3(_input.X, _input.Y, _input.Z);
            return _output;
        }
        public static Vector2 convertToVector2(this IPoint3 _input)
        {
            Vector2 _output = new Vector2(_input.X, _input.Y);
            return _output;
        }
    }
}
