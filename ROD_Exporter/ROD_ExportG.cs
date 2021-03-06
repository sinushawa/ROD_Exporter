﻿using System;
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
using Pose = ROD_core.Graphics.Animation.Pose;
using Joint=ROD_core.Graphics.Animation.Joint;
using Skeleton = ROD_core.Graphics.Animation.Skeleton;
using DualQuaternion = ROD_core.Mathematics.DualQuaternion;
using CurveHandle = ROD_core.Graphics.Animation.CurveHandle;
using BezierCurveFitting;
using MoreLinq;

namespace ROD_Exporter
{

    public class ROD_ExportG
    {
        public static IGlobal maxGlobal;
        public static IInterface14 maxInterface;

        public static List<uint> SelectedNodes = new List<uint>();
        public static Semantic semantic;
        private static IMatrix3 _leftHanded;
        private static IGMatrix _GleftHanded;

        public ROD_ExportG()
        {
            maxGlobal = Autodesk.Max.GlobalInterface.Instance;
            maxInterface = maxGlobal.COREInterface14;
            IPoint3 U = maxGlobal.Point3.Create(1.0, 0.0, 0.0);
            IPoint3 V = maxGlobal.Point3.Create(0.0, 0.0, 1.0);
            IPoint3 N = maxGlobal.Point3.Create(0.0, -1.0, 0.0);
            IPoint3 T = maxGlobal.Point3.Create(0.0, 0.0, 0.0);
            _leftHanded = maxGlobal.Matrix3.Create(U, V, N, T);
            _GleftHanded = maxGlobal.GMatrix.Create(_leftHanded);
        }

        private static List<IINode> GetSelection()
        {
            List<IINode> selectedNodes = new List<IINode>();
            for (int i = 0; i < maxInterface.SelNodeCount; i++)
            {
                selectedNodes.Add(maxInterface.GetSelNode(i));
            }
            return selectedNodes;
        }

        public static bool Export_To(string _filepath)
        {
            List<IINode> nodes = GetSelection();

            foreach (IINode _node in nodes)
            {
                
                IIDerivedObject _gObject = (IIDerivedObject)_node.ObjectRef;
                IClass_ID classID = maxGlobal.Class_ID.Create((uint)BuiltInClassIDA.TRIOBJ_CLASS_ID, 0);
                ITriObject _mObject = (ITriObject)_gObject.ObjRef.ConvertToType(0, classID);
                IMesh _mMesh = _mObject.Mesh;
                _mMesh.BuildNormals();
                IIDerivedObject theObj = (IIDerivedObject)_node.ObjectRef;
                for (int m = 0; m < theObj.Modifiers.Count; m++)
                {
                    IModifier theModifier = theObj.GetModifier(m);
                    if (theModifier.ClassName == "Skin")
                    {

                        IISkin _skin = (IISkin)theModifier.GetInterface((InterfaceID)(0x00010000));
                        IISkinContextData _skinContext = _skin.GetContextInterface(_node);
                        ComputeVertexData(_mMesh, _skinContext, semantic, _filepath);
                    }
                }
                

            }
            return true;
        }
        public static bool ExportSkeleton(string _filenameSkeleton)
        {
            List<IINode> nodes = GetSelection();

            foreach (IINode _node in nodes)
            {
                IIDerivedObject theObj = (IIDerivedObject)_node.ObjectRef;
                IInterval interval = maxGlobal.Interval.Create();
                interval.SetInfinite();
                maxGlobal.IGameInterface.InitialiseIGame(false);
                for (int m = 0; m < theObj.Modifiers.Count; m++)
                {
                    IModifier theModifier = theObj.GetModifier(m);
                    if (theModifier.ClassName == "Skin")
                    {

                        IISkin _skin = (IISkin)theModifier.GetInterface((InterfaceID)(0x00010000));
                        IINode _bone = _skin.GetBone(0);

                        int nbBones = _skin.NumBones;
                        List<string> boneName = new List<string>();
                        for (int b = 0; b < nbBones; b++)
                        {
                            boneName.Add(_skin.GetBone(b).Name);
                        }
                        #region create bindPose World
                        Pose bindPose = new Pose("bindPose");
                        BuildBind(_bone, -1, bindPose);
                        #endregion

                        Skeleton skelete = new Skeleton("skelete", bindPose);
                        skelete.saveToFile(_filenameSkeleton);
                    }
                }
            }
            return true;
        }
        public static bool ExportAnimation(int[] _frames, string _filename, int _samplingRate)
        {

            List<IINode> nodes = GetSelection();

            foreach (IINode _node in nodes)
            {
                IIDerivedObject theObj = (IIDerivedObject)_node.ObjectRef;
                IInterval interval = maxGlobal.Interval.Create();
                interval.SetInfinite();
                maxGlobal.IGameInterface.InitialiseIGame(false);
                for (int m = 0; m < theObj.Modifiers.Count; m++)
                {
                    IModifier theModifier = theObj.GetModifier(m);
                    if (theModifier.ClassName == "Skin")
                    {

                        IISkin _skin = (IISkin)theModifier.GetInterface((InterfaceID)(0x00010000));
                        IINode _bone = _skin.GetBone(0);
                        
                        int nbBones = _skin.NumBones;
                        List<string> boneName = new List<string>();
                        for (int b = 0; b < nbBones; b++)
                        {
                            boneName.Add(_skin.GetBone(b).Name);
                        }
                        #region create bindPose World
                        Pose bindPose = new Pose("bindPose");
                        BuildBind(_bone, -1, bindPose);
                        #endregion

                        ROD_core.Graphics.Animation.Clip_Skinning clip = new ROD_core.Graphics.Animation.Clip_Skinning();
                        clip.sequencesData = new List<Pose>();
                        clip.sequencesTiming = new List<TimeSpan>();
                        for (int f = 0; f < _frames.Length; f++)
                        {
                            // create Pose at frame (_frame)
                            Pose _pose = new Pose("frame" + _frames[f].ToString());
                            BuildLJoint(_bone, -1, _pose, _frames, f, _samplingRate, bindPose);
                            clip.sequencesData.Add(_pose);
                            clip.sequencesTiming.Add(TimeSpan.FromSeconds(_frames[f] / 30));
                        }

                        clip.saveToFile(_filename);
                    }
                }
            }
            return true;
        }
        public static void BuildBind(IINode _node, int _parentId, Pose _pose)
        {
            DualQuaternion DQ = GetBoneBindDQ(_node);
            Joint _joint = new Joint(_pose.joints.Count, _node.Name, _parentId, DQ, DualQuaternion.Identity);
            _pose.joints.Add(_joint);
            int childrensNb = _node.NumberOfChildren;
            for (int i = 0; i < childrensNb; i++)
            {
                if (!_node.GetChildNode(i).Name.EndsWith("Nub"))
                {
                    BuildBind(_node.GetChildNode(i), _joint.id, _pose);
                }
            }
        }

        public static void BuildLJoint(IINode _node, int _parentId, Pose _pose,int[] _frames, int _f, int _samplingRate, Pose _bindPose)
        {
            DualQuaternion LDQ = GetBoneLocalDQ(_node, _frames, _f);
            CurveHandle curve = GetCurve(_node, _frames, _f, _samplingRate);
            Joint _joint = new Joint(_pose.joints.Count, _node.Name, _parentId, DualQuaternion.Identity, LDQ, curve);
            _pose.joints.Add(_joint);
            int childrensNb = _node.NumberOfChildren;
            for (int i = 0; i < childrensNb; i++)
            {
                if (!_node.GetChildNode(i).Name.EndsWith("Nub"))
                {
                    BuildLJoint(_node.GetChildNode(i), _joint.id, _pose, _frames, _f, _samplingRate, _bindPose);
                }
            }
        }

        public static DualQuaternion GetBoneBindDQ(IINode _node)
        {
            return GetBoneWorldDQ(_node, 0);
        }
        public static DualQuaternion GetBoneWorldDQ(IINode _node, int _frame)
        {
            IInterval interval = maxGlobal.Interval.Create();
            interval.SetInfinite();
            IIGameNode GNode = maxGlobal.IGameInterface.GetIGameNode(_node);
            int _ticks_per_frame = maxGlobal.TicksPerFrame;
            IGMatrix _node_Gmatrix = GNode.GetWorldTM(_frame * _ticks_per_frame);
            DualQuaternion DQ = _node_Gmatrix.convertToDQ();

            return DQ;
        }
        public static DualQuaternion GetBoneLocalDQ(IINode _node,int[] _frames, int _f)
        {
            IINode _parent_node = _node.ParentNode;
            IIGameNode _GNode = maxGlobal.IGameInterface.GetIGameNode(_node);
            IIGameNode _parentGNode = maxGlobal.IGameInterface.GetIGameNode(_parent_node);
            int _ticks_per_frame = maxGlobal.TicksPerFrame;

            // node Local Transform
            IGMatrix _node_LGmatrix = _GNode.GetLocalTM(_frames[_f] * _ticks_per_frame);
            DualQuaternion _node_LDQ = _node_LGmatrix.convertToDQ(Transformation.Rotation);
            _node_LDQ.Normalize();
            IGMatrix _node_BLGmatrix = _GNode.GetLocalTM(0);
            DualQuaternion _node_BLDQ = _node_BLGmatrix.convertToDQ(Transformation.Rotation);
            _node_BLDQ.Normalize();
            DualQuaternion _node_LTDQL = _node_LDQ * DualQuaternion.Conjugate(_node_BLDQ);

            return _node_LTDQL;
        }
        public static CurveHandle GetCurve(IINode _node, int[] _frames, int _f, int _samplingRate)
        {
            CurveHandle result = new CurveHandle(Vector2.Zero, Vector2.Zero);
            IIGameNode _GNode = maxGlobal.IGameInterface.GetIGameNode(_node);
            int _ticks_per_frame = maxGlobal.TicksPerFrame;
            float delta=0.0f;
            if (_f>0)
            {
                List<Vector3> _samples = new List<Vector3>();
                int nb_of_frames = _frames[_f ] - _frames[_f-1];
                for (int i =  _frames[_f-1]; i <= _frames[_f]; i += _samplingRate)
                {
                    float proportion = 1.0f - ((_frames[_f] - i) / (float)(nb_of_frames));
                    IGMatrix _node_LGmatrix = _GNode.GetLocalTM(i * _ticks_per_frame);
                    DualQuaternion _node_LDQ = _node_LGmatrix.convertToDQ(Transformation.Rotation);
                    _samples.Add(new Vector3(proportion, Math.Abs(_node_LDQ.RollPitchYaw.X)+Math.Abs(_node_LDQ.RollPitchYaw.Y)+Math.Abs(_node_LDQ.RollPitchYaw.Z),0));
                }
                List<BezierCurveFitting.BezierPoint> points = BezierCurveFitting.FitCurves.FitCurveBy(BezierCurveFitting.Method.Length, _samples.ToArray(), 0.01f, 2);
                delta = points[1].point.Y - points[0].point.Y;
                float val = points[1].foreHandle.Value.X;
                if (delta != 0)
                {
                    val = (points[1].point.Y - points[1].foreHandle.Value.Y) / delta;
                }
                
                result.easeIn = new Vector2(points[1].foreHandle.Value.X, val);
                
            }
            else if (_f == 0)
            {
                result.easeIn = new Vector2(0.0f, 0.0f);
            }
            if (_f < _frames.Length-1)
            {
                List<Vector3> _samples = new List<Vector3>();
                int nb_of_frames = _frames[_f+1] - _frames[_f];
                for (int i = _frames[_f]; i <= _frames[_f+1]; i += _samplingRate)
                {
                    float proportion = 1.0f - ((_frames[_f+1] - i) / (float)(nb_of_frames));
                    IGMatrix _node_LGmatrix = _GNode.GetLocalTM(i * _ticks_per_frame);
                    DualQuaternion _node_LDQ = _node_LGmatrix.convertToDQ(Transformation.Rotation);
                    _samples.Add(new Vector3(proportion, Math.Abs(_node_LDQ.RollPitchYaw.X) + Math.Abs(_node_LDQ.RollPitchYaw.Y) + Math.Abs(_node_LDQ.RollPitchYaw.Z), 0));
                }
                List<BezierCurveFitting.BezierPoint> points = BezierCurveFitting.FitCurves.FitCurveBy(BezierCurveFitting.Method.Length, _samples.ToArray(), 0.01f, 2);
                delta = points[1].point.Y - points[0].point.Y;
                float val = points[0].afterHandle.Value.X;
                if (delta != 0)
                {
                    val = (points[1].point.Y - points[0].afterHandle.Value.Y) / delta;
                }
                result.easeOut = new Vector2(points[0].afterHandle.Value.X, val);
            }
            else if (_f == _frames.Length-1)
            {
                result.easeOut = new Vector2(1f, 1f);
            }
            return result;
        }
        
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

        public static void ComputeVertexData(IMesh _mMesh, IISkinContextData _skin, Semantic _semantic, string _filepath)
        {
            Dictionary<int, VertexUndivided> verticesFullData = new Dictionary<int, VertexUndivided>();
            List<FaceData> facesFullData = new List<FaceData>();
            List<List<VertexDivided>> verticesParsedData = new List<List<VertexDivided>>();

            IList<IFace> faces = _mMesh.Faces;
            IList<ITVFace> Tfaces = _mMesh.TvFace;
            IList<IPoint3> vertices = _mMesh.Verts;
            IList<IPoint3> Tvertices = _mMesh.TVerts;
            
            /*
            foreach (IPoint3 _v in vertices)
            {
                float temp = _v.Y;
                _v.Y = -_v.Z;
                _v.Z = temp;
            }
            */
            for (int _fID = 0; _fID < faces.Count; _fID++)
            {
                FaceData _face = new FaceData((int)faces[_fID].SmGroup);

                // vectors are inverted to make up for max being clockwise
                Vector3 A_B = vertices[(int)faces[_fID].GetVert(1)].convertToVector3() - vertices[(int)faces[_fID].GetVert(0)].convertToVector3();
                Vector3 A_C = vertices[(int)faces[_fID].GetVert(2)].convertToVector3() - vertices[(int)faces[_fID].GetVert(0)].convertToVector3();
                Vector3 U = Tvertices[(int)Tfaces[_fID].GetTVert(1)].convertToVector3() - Tvertices[(int)Tfaces[_fID].GetTVert(0)].convertToVector3();
                Vector3 V = Tvertices[(int)Tfaces[_fID].GetTVert(2)].convertToVector3() - Tvertices[(int)Tfaces[_fID].GetTVert(0)].convertToVector3();

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
                        int nbBonesA=_skin.GetNumAssignedBones((int)_v.ID);
                        List<int> bonesID = new List<int>();
                        List<float> bonesWeights = new List<float>();
                        for (int b = 0; b < 4; b++)
                        {
                            if(nbBonesA<b+1)
                            {
                                bonesID.Add(0);
                                bonesWeights.Add(0);
                            }
                            else
                            {
                                bonesID.Add(_skin.GetAssignedBone((int)_v.ID, b));
                                bonesWeights.Add(_skin.GetBoneWeight((int)_v.ID, b));
                            }
                        }
                        _v.bonesID = new ROD_core.BoneIndices(bonesID[0], bonesID[1], bonesID[2], bonesID[3]);
                        _v.bonesWeights = new Vector4(bonesWeights[0], bonesWeights[1], bonesWeights[2], bonesWeights[3]);
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
                facesFullData[_faceID].vertices.Reverse();
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
                    VertexDivided _newVertex = new VertexDivided();
                    _newVertex.pos = verticesFullData[_vertex].pos;

                    _newVertex.UV.X = verticesFullData[_vertex].faceInfo.Where(x => x.ID == _faceID).FirstOrDefault().UV.X;
                    _newVertex.UV.Y = 1 - verticesFullData[_vertex].faceInfo.Where(x => x.ID == _faceID).FirstOrDefault().UV.Y;
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
                    _newVertex.bonesID = verticesFullData[_vertex].bonesID;
                    _newVertex.bonesWeights = verticesFullData[_vertex].bonesWeights;
                    IndexBuffer.Add(VertexBuffer.Count);
                    VertexBuffer.Add(_newVertex);
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
            Type dv = DynamicVertex.CreateVertex(_semantic);
            FieldInfo[] PI = dv.GetFields();
            
            foreach (VertexDivided vd in VertexBuffer)
            {
                if (mesh._boundingBox.Minimum == null)
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
                List<object> vertexData = new List<object>();
                for (int i = 0; i < PI.Length; i++)
                {
                    string fieldSemantic = ((InputElementAttribute)PI[i].GetCustomAttributes(true).First()).Semantic;
                    vertexData.Add(vd.GetSemanticObject(fieldSemantic));
                }
                object[] obj = vertexData.ToArray();
                //object[] obj = new object[] { vd.pos, vd.normal, vd.UV, vd.binormal, vd.bonesID, vd.bonesWeights, vd.tangent };
                //object[] obj = new object[] { vd.pos, vd.normal, vd.UV, vd.binormal, vd.tangent };
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
        public ROD_core.BoneIndices bonesID;
        public Vector4 bonesWeights;

        public VertexUndivided()
        {
            faceInfo = new List<PerFaceInfo>();
        }
    }
    public class SemanticMatch : Attribute
    {
        public SemanticMatch(Semantic _inputSemantic)
        {
            this.inputSemantic = _inputSemantic;
        }
        private Semantic inputSemantic;
        public Semantic InputSemantic
        {
            get { return inputSemantic; }
            set { inputSemantic = value; }
        }
    }
    public struct VertexDivided
    {
        [SemanticMatch(Semantic.POSITION)]
        public Vector3 pos;
        [SemanticMatch(Semantic.NORMAL)]
        public Vector3 normal;
        [SemanticMatch(Semantic.TEXCOORD)]
        public Vector2 UV;
        [SemanticMatch(Semantic.BINORMAL)]
        public Vector3 binormal;
        [SemanticMatch(Semantic.TANGENT)]
        public Vector3 tangent;
        [SemanticMatch(Semantic.BLENDINDICES)]
        public ROD_core.BoneIndices bonesID;
        [SemanticMatch(Semantic.BLENDWEIGHT)]
        public Vector4 bonesWeights;
    }


    public enum Transformation
    {
        None  = (1 << 0),
        Rotation = (1 << 1),
        Translation = (1 << 2),
        Scaling = (1 << 3),
        All = Rotation | Translation | Scaling
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
        public static Matrix convertTo(this IMatrix3 _input)
        {
            Matrix _output = new Matrix(_input.GetRow(0).X, _input.GetRow(0).Y, _input.GetRow(0).Z, 0, _input.GetRow(1).X, _input.GetRow(1).Y, _input.GetRow(1).Z, 0, _input.GetRow(2).X, _input.GetRow(2).Y, _input.GetRow(2).Z, 0, 0, 0, 0, 1);
            return _output;
        }
        public static Matrix convertTo(this IGMatrix _input)
        {
            Matrix _output = new Matrix(_input.GetRow(0).X, _input.GetRow(0).Y, _input.GetRow(0).Z, 0, _input.GetRow(1).X, _input.GetRow(1).Y, _input.GetRow(1).Z, 0, _input.GetRow(2).X, _input.GetRow(2).Y, _input.GetRow(2).Z, 0, _input.GetRow(3).X, _input.GetRow(3).Y, _input.GetRow(3).Z, 1);
            return _output;
        }
        public static DualQuaternion convertToDQ(this IGMatrix _input)
        {
            Matrix _matrix = new Matrix(_input.GetRow(0).X, _input.GetRow(0).Y, _input.GetRow(0).Z, 0, _input.GetRow(1).X, _input.GetRow(1).Y, _input.GetRow(1).Z, 0, _input.GetRow(2).X, _input.GetRow(2).Y, _input.GetRow(2).Z, 0, _input.GetRow(3).X, _input.GetRow(3).Y, _input.GetRow(3).Z, 1);
            Quaternion _node_LQ;
            Vector3 _node_LS;
            Vector3 _node_LT;
            _matrix.Decompose(out _node_LS, out _node_LQ, out _node_LT);
            DualQuaternion _output = new DualQuaternion(_node_LQ, _node_LT);
            return _output;
        }
        public static DualQuaternion convertToDQ(this IGMatrix _input, Transformation _transformation)
        {
            Matrix _matrix = new Matrix(_input.GetRow(0).X, _input.GetRow(0).Y, _input.GetRow(0).Z, 0, _input.GetRow(1).X, _input.GetRow(1).Y, _input.GetRow(1).Z, 0, _input.GetRow(2).X, _input.GetRow(2).Y, _input.GetRow(2).Z, 0, _input.GetRow(3).X, _input.GetRow(3).Y, _input.GetRow(3).Z, 1);
            Quaternion _node_LQ;
            Vector3 _node_LS;
            Vector3 _node_LT;
            _matrix.Decompose(out _node_LS, out _node_LQ, out _node_LT);
            DualQuaternion _output = DualQuaternion.Identity;
            if (_transformation == Transformation.All)
            {
                _output = new DualQuaternion(_node_LQ, _node_LT);
            }
            if (_transformation == Transformation.Rotation)
            {
                _output = new DualQuaternion(_node_LQ, Vector3.Zero);
            }
            if (_transformation == Transformation.Translation)
            {
                _output = new DualQuaternion(Quaternion.Identity, _node_LT);
            }
            return _output;
        }
        public static Quaternion convertTo(this IQuat _input)
        {
            return new Quaternion(_input.X, _input.Y, _input.Z, _input.W);
        }
        public static IntPtr IntPtrFromFloat(float f)
        {
            unsafe
            {
                return (*(IntPtr*)&f);
            }
        }

        public static Object GetSemanticObject(this VertexDivided value, string _semantic)
        {
            Type type = value.GetType();

            // Get fieldinfo for this type
            FieldInfo[] fieldInfos = type.GetFields();
            Object ret = null;
            foreach (FieldInfo fi in fieldInfos)
            {
                Semantic attrib = (fi.GetCustomAttributes(typeof(SemanticMatch), false) as SemanticMatch[]).First().InputSemantic;
                if(attrib.ToString() == _semantic)
                {
                    ret = fi.GetValue(value);
                    return ret;
                }
            }

            // Return the first if there was a match.
            try
            {
                return ret;
            }
            catch (Exception ex)
            {
                throw new SystemException(ex.Message);
            }
        }
    }
}
