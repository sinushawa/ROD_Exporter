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
using System.IO;
using System.Diagnostics;

namespace ROD_Exporter
{

    public class ROD_ExportG
    {
        public IGlobal maxGlobal;
        public IInterface13 maxInterface;

        public static List<uint> SelectedNodes = new List<uint>();
        public static Dictionary<IINode, ROD_core.Graphics.Animation.Joint> NodeBJointDic = new Dictionary<IINode, ROD_core.Graphics.Animation.Joint>();
        public static Dictionary<IINode, ROD_core.Graphics.Animation.Joint> NodeJointDic = new Dictionary<IINode, ROD_core.Graphics.Animation.Joint>();
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
            ROD_ExportG r = new ROD_ExportG();
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
            ROD_ExportG r = new ROD_ExportG();
            
            foreach (uint _handle in SelectedNodes)
            {
                IINode _node = r.maxInterface.GetINodeByHandle(_handle);
                IIDerivedObject theObj = (IIDerivedObject)_node.ObjectRef;
                for (int m = 0; m < theObj.Modifiers.Count; m++)
                {
                    IModifier theModifier = theObj.GetModifier(m);
                    if (theModifier.ClassName == "Skin")
                    {
                        IISkin _skin = (IISkin)theModifier.GetInterface((InterfaceID)(0x00010000));
                        IINode _bone = _skin.GetBone(0);
                        // create bindPose
                        using (StreamWriter writer =  new StreamWriter("Bind.txt"))
                        {
                            NodeBJointDic = new Dictionary<IINode, ROD_core.Graphics.Animation.Joint>();
                            ROD_core.Mathematics.DualQuaternion BDQ = GetBoneBindDQ(_bone, _skin, r, writer);
                            ROD_core.Graphics.Animation.Joint Bjoint = new ROD_core.Graphics.Animation.Joint((int)_bone.GetHashCode(), _bone.Name, null, BDQ);
                            NodeBJointDic.Add(_bone, Bjoint);
                            Bjoint = BuildBind(Bjoint, _skin, r, writer);
                        }

                        using (StreamWriter writer = new StreamWriter("Pose2.txt"))
                        {
                            // create Pose at frame (_frame)
                            NodeJointDic = new Dictionary<IINode, ROD_core.Graphics.Animation.Joint>();
                            ROD_core.Mathematics.DualQuaternion DQ = GetBoneLocalDQ(_bone, _frame, r, writer);
                            ROD_core.Graphics.Animation.Joint joint = new ROD_core.Graphics.Animation.Joint((int)_bone.GetHashCode(), _bone.Name, null, DQ);
                            NodeJointDic.Add(_bone, joint);
                            joint = BuildJoint(joint, _frame, r, writer);
                        }
                        IISkinContextData _skinContext = _skin.GetContextInterface(_node);
                    }
                }
            }
            return true;
        }
        public static ROD_core.Graphics.Animation.Joint BuildBind(ROD_core.Graphics.Animation.Joint joint, IISkin _skin, ROD_ExportG r, StreamWriter writer)
        {
            IINode thisNode = NodeBJointDic.Where(x => x.Value == joint).Select(y => y.Key).First();
            int childrensNb = thisNode.NumberOfChildren;
            for (int i = 0; i < childrensNb; i++)
            {
                IINode child = thisNode.GetChildNode(i);
                ROD_core.Mathematics.DualQuaternion DQ = GetBoneBindDQ(child, _skin, r, writer);
                ROD_core.Graphics.Animation.Joint newJoint = new ROD_core.Graphics.Animation.Joint((int)child.GetHashCode(), child.Name, joint, DQ);
                NodeBJointDic.Add(child, newJoint);
                joint.children.Add(BuildBind(newJoint, _skin, r, writer));
            }
            return joint;
        }
        public static ROD_core.Graphics.Animation.Joint BuildJoint(ROD_core.Graphics.Animation.Joint joint, int _frame, ROD_ExportG r, StreamWriter writer)
        {
            IINode thisNode = NodeJointDic.Where(x => x.Value == joint).Select(y => y.Key).First();
            int childrensNb = thisNode.NumberOfChildren;
            for (int i = 0; i < childrensNb; i++)
            {
                IINode child = thisNode.GetChildNode(i);
                ROD_core.Mathematics.DualQuaternion DQ = GetBoneLocalDQ(child, _frame, r, writer);
                ROD_core.Graphics.Animation.Joint newJoint = new ROD_core.Graphics.Animation.Joint((int)child.GetHashCode(), child.Name, joint, DQ);
                NodeJointDic.Add(child, newJoint);
                joint.children.Add(BuildJoint(newJoint, _frame, r, writer));
            }
            return joint;
        }
        public static ROD_core.Mathematics.DualQuaternion GetBoneBindDQ(IINode _node, IISkin _skin, ROD_ExportG r, StreamWriter writer)
        {
            IMatrix3 _matrix = r.maxGlobal.Matrix3.Create();
            _skin.GetBoneInitTM(_node, _matrix, false);
            IPoint3 _local_Translation = r.maxGlobal.Point3.Create();
            IQuat _local_Rotation = r.maxGlobal.Quat.Create();
            IPoint3 _local_Scale = r.maxGlobal.Point3.Create();
            r.maxGlobal.DecomposeMatrix(_matrix, _local_Translation, _local_Rotation, _local_Scale);
            writer.WriteLine(_node.Name + " Translation:X:" + _local_Translation.X.ToString() + " Y:" + _local_Translation.Y.ToString() + " Z:" + _local_Translation.Z.ToString() + " // Rotation:X:" + _local_Rotation.X.ToString()+ " Y:"+_local_Rotation.Y.ToString() + " Z:" +_local_Rotation.Z.ToString() + " W:" +_local_Rotation.W.ToString() );
            ROD_core.Mathematics.DualQuaternion DQ = new ROD_core.Mathematics.DualQuaternion(new Quaternion(_local_Rotation.X, _local_Rotation.Y, _local_Rotation.Z, _local_Rotation.W), new Vector3(_local_Translation.X, _local_Translation.Y, _local_Translation.Z));
            return DQ;
        }
        public static ROD_core.Mathematics.DualQuaternion GetBoneLocalDQ(IINode _node, int _frame, ROD_ExportG r, StreamWriter writer)
        {
            IInterval interval = r.maxGlobal.Interval.Create();
            interval.SetInfinite();
            int _ticks_per_frame = r.maxGlobal.TicksPerFrame;
            IMatrix3 _node_matrix = _node.GetNodeTM(_frame * _ticks_per_frame, interval);
            IMatrix3 _parent_matrix = _node.GetParentTM(_frame * _ticks_per_frame);
            _parent_matrix.Invert();
            IMatrix3 _local_matrix = _node_matrix.Multiply(_parent_matrix);
            IPoint3 _local_Translation = r.maxGlobal.Point3.Create();
            IQuat _local_Rotation = r.maxGlobal.Quat.Create();
            IPoint3 _local_Scale = r.maxGlobal.Point3.Create();
            r.maxGlobal.DecomposeMatrix(_local_matrix, _local_Translation, _local_Rotation, _local_Scale);
            writer.WriteLine(_node.Name + " Translation:X:" + _local_Translation.X.ToString() + " Y:" + _local_Translation.Y.ToString() + " Z:" + _local_Translation.Z.ToString() + " // Rotation:X:" + _local_Rotation.X.ToString() + " Y:" + _local_Rotation.Y.ToString() + " Z:" + _local_Rotation.Z.ToString() + " W:" + _local_Rotation.W.ToString());
            ROD_core.Mathematics.DualQuaternion DQ = new ROD_core.Mathematics.DualQuaternion(new Quaternion(_local_Rotation.X, _local_Rotation.Y, _local_Rotation.Z, _local_Rotation.W), new Vector3(_local_Translation.X, _local_Translation.Y, _local_Translation.Z));
            ROD_core.Graphics.Animation.Joint _bindJoint = NodeBJointDic.Where(x => x.Key.Name == _node.Name).Select(x => x.Value).First();
            ROD_core.Mathematics.DualQuaternion _bindConjuguate = ROD_core.Mathematics.DualQuaternion.Conjugate(_bindJoint.localRotationTranslation);
            DQ = DQ * _bindConjuguate;
            Debug.WriteLine(ROD_core.Mathematics.DualQuaternion.GetTranslation(DQ));
            writer.WriteLine(_node.Name + " Real:X:" + DQ.real.X.ToString() + " Y:" + DQ.real.Y.ToString() + " Z:" + DQ.real.Z.ToString() + " W:" + DQ.real.W.ToString() + " // Dual:X:" + DQ.dual.X.ToString() + " Y:" + DQ.dual.Y.ToString() + " Z:" + DQ.dual.Z.ToString() + " W:" + DQ.dual.W.ToString());

            return DQ;
        }
        static IntPtr IntPtrFromFloat(float f)
        {
            unsafe
            {
                return (*(IntPtr*)&f);
            }
        }

        public ROD_ExportG()
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
        public ROD_core.BoneIndices bonesID;
        public Vector4 bonesWeights;
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
