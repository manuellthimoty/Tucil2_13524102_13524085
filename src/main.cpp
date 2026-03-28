#include <bits/stdc++.h>

using namespace std;
using ll = long long;

// struct untuk point
struct VectorV{
    float x,y,z;
};
// face untuk simpan indeks dari point
struct Face{
    int i,j,k;
};

int countV = 0;
int finalDepth = 0;
int maxDepth;

vector<VectorV> vertices;
vector<Face> faces;

vector<vector<int>> resFaceIds;

vector<VectorV> voxelVertices;
vector<Face> voxelFaces;

vector<ll> statTerbentuk;
vector<ll> statTidakDitelusuri;

bool loadObject(const string& path, vector<VectorV> &resVertices, vector<Face> &resFaces){
    ifstream file(path);
    if(!file.is_open()){
        cout << "Gagal Membuka file" << endl;
        return false;
    }

    string line;
    while(getline(file,line)){
        stringstream ss(line);
        string pref;
        ss >> pref;
        string sisa;

        if(pref == "#" ||  pref == "g") continue;
       
        if(pref == "v"){
            VectorV vertex;
            if(ss >> vertex.x >> vertex.y >> vertex.z){
                if(ss >> sisa) {cout << "Gagal : Format data vertex pada file.obj tidak sesuai" << endl; return false;}
                resVertices.push_back(vertex);
            }
            else{
                cout << "Format vertex tidak sesuai " << endl;
                return false;
            }
        }
        else if (pref == "f"){
            int first,second,third;
            if(ss >> first >> second >> third){
                if(ss >> sisa) {cout << "Gagal : Format data face pada file.obj tidak sesuai" << endl; return false;}
                Face face;
                face.i = first-1;
                face.j = second-1;
                face.k = third-1;
                resFaces.push_back(face);
            }
            else{
                cout << "Format face tidak sesuai" << endl;
                return false;
            }
            
        }
        // else return false;
    }
    return true;
}
struct AABB{
    VectorV minPoint; VectorV maxPoint;
    bool intersect(const AABB& other) const {
        return (minPoint.x <= other.maxPoint.x && maxPoint.x >= other.minPoint.x
        && minPoint.y <= other.maxPoint.y && maxPoint.y >= other.minPoint.y
        && minPoint.z <= other.maxPoint.z && maxPoint.z >= other.minPoint.z);
    }
};


// vektor bantuan untuk SAT
VectorV subVec(const VectorV& a, const VectorV& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
VectorV crossProduct(const VectorV& a, const VectorV& b) {return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x}; }
float dotProduct(const VectorV& a, const VectorV& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }


// CEK SEGITIGA VS KOTAK VOXEL 
bool checkTriangleBox(const VectorV& boxMin, const VectorV& boxMax, const VectorV& v0, const VectorV& v1, const VectorV& v2) {
    // 1. Cek Cepat (AABB vs AABB)
    float triMinX = min({v0.x, v1.x, v2.x}), triMaxX = max({v0.x, v1.x, v2.x});
    float triMinY = min({v0.y, v1.y, v2.y}), triMaxY = max({v0.y, v1.y, v2.y});
    float triMinZ = min({v0.z, v1.z, v2.z}), triMaxZ = max({v0.z, v1.z, v2.z});
    
    if (triMaxX < boxMin.x || triMinX > boxMax.x) return false;
    if (triMaxY < boxMin.y || triMinY > boxMax.y) return false;
    if (triMaxZ < boxMin.z || triMinZ > boxMax.z) return false;

    // Cek Bidang Segitiga 
    // Hitung normal segitiga
    VectorV edge0 = subVec(v1, v0);
    VectorV edge1 = subVec(v2, v1);
    VectorV normal = crossProduct(edge0, edge1);
    
    // Cari titik di kotak yang paling jauh searah normal dan berlawanan arah normal
    VectorV vmin, vmax;
    if(normal.x > 0) {vmin.x = boxMin.x; vmax.x = boxMax.x; } else { vmin.x = boxMax.x; vmax.x = boxMin.x; }
    if(normal.y > 0) {vmin.y = boxMin.y; vmax.y = boxMax.y; } else { vmin.y = boxMax.y; vmax.y = boxMin.y; }
    if(normal.z > 0) {vmin.z = boxMin.z; vmax.z = boxMax.z; } else { vmin.z = boxMax.z; vmax.z = boxMin.z; }
    
    // Jika seluruh kotak ada di luar bidang segitiga
    if(dotProduct(normal, subVec(vmin, v0)) > 0) return false;
    if(dotProduct(normal, subVec(vmax, v0)) < 0) return false;
    
    return true;
}
class Octree{
    public :
        AABB bound;// untuk bound
        bool isLeaf;// bool apakah ini daun
        int curDepth;// kedalaman sekarang
        vector<int> faceIds; // faceId yg disimpan di node ini
        Octree* children[8]; // ke8 anaknya

        // konstruktor
        Octree(AABB b, int d){
            bound = b;
            curDepth = d;
            isLeaf = true;
            for(int i = 0 ; i < 8 ; i++) children[i] = nullptr;
            if(d > 0) statTerbentuk[d]++;
        }

        void subdivide(){
            float x_mid = (bound.minPoint.x + bound.maxPoint.x) / 2.0f;
            float y_mid = (bound.minPoint.y + bound.maxPoint.y) / 2.0f;
            float z_mid = (bound.minPoint.z + bound.maxPoint.z) / 2.0f;
            children[0] = new Octree({{bound.minPoint.x, bound.minPoint.y, bound.minPoint.z}, {x_mid, y_mid, z_mid}}, curDepth + 1);
            children[1] = new Octree({{x_mid, bound.minPoint.y, bound.minPoint.z}, {bound.maxPoint.x, y_mid, z_mid}}, curDepth + 1);
            children[2] = new Octree({{bound.minPoint.x, bound.minPoint.y, z_mid}, {x_mid, y_mid, bound.maxPoint.z}}, curDepth + 1);
            children[3] = new Octree({{x_mid, bound.minPoint.y, z_mid}, {bound.maxPoint.x, y_mid, bound.maxPoint.z}}, curDepth + 1);

            // Atas (Y mid ke Y maxPoint)
            children[4] = new Octree({{bound.minPoint.x, y_mid, bound.minPoint.z}, {x_mid, bound.maxPoint.y, z_mid}}, curDepth + 1);
            children[5] = new Octree({{x_mid, y_mid, bound.minPoint.z}, {bound.maxPoint.x, bound.maxPoint.y, z_mid}}, curDepth + 1);
            children[6] = new Octree({{bound.minPoint.x, y_mid, z_mid}, {x_mid, bound.maxPoint.y, bound.maxPoint.z}}, curDepth + 1);
            children[7] = new Octree({{x_mid, y_mid, z_mid}, {bound.maxPoint.x, bound.maxPoint.y, bound.maxPoint.z}}, curDepth + 1);

            isLeaf = false;
        }

        void buildOctree(const vector<int>& inputFaceId, const vector<Face>& globalFaces, const vector<VectorV>& globalVertices, int maxDepth){
            if(curDepth >= maxDepth){
                faceIds = inputFaceId;
                return;
            }

            subdivide();

            for(int faceId : inputFaceId){
                VectorV v0 = globalVertices[globalFaces[faceId].i];
                VectorV v1 = globalVertices[globalFaces[faceId].j];
                VectorV v2 = globalVertices[globalFaces[faceId].k];

                for(int i = 0 ; i < 8 ; i++){
                    // yang sebelumnya AABB jadi pengecekan triangle
                    if(checkTriangleBox(children[i]->bound.minPoint, children[i]->bound.maxPoint, v0, v1, v2)) {
                        children[i]->faceIds.push_back(faceId);
                    }
                }
            }

            for(int i = 0  ; i < 8 ; i++){
                if(!children[i]->faceIds.empty()){
                    children[i]->buildOctree(children[i]->faceIds,globalFaces,globalVertices,maxDepth);
                }
                else{
                    int anakDepth = children[i]->curDepth;
                    if(anakDepth <= maxDepth) statTidakDitelusuri[anakDepth]++;
                }
            }

            faceIds.clear();
        }

};

void buildResult(Octree* node){
    if(!node->isLeaf){
        for(int i = 0 ; i < 8 ; i++){
            if(node->children[i] != nullptr){
                buildResult(node->children[i]);
            }
        }
    }
    else{
        finalDepth = max(finalDepth,node->curDepth);
        if (node->faceIds.empty()) return;

        VectorV minP = node->bound.minPoint;
        VectorV maxP = node->bound.maxPoint;
        
        int curIndex = voxelVertices.size(); 

        // 8 titik kubus
        voxelVertices.push_back({minP.x, minP.y, minP.z}); // 0: Kiri Bawah Depan
        voxelVertices.push_back({maxP.x, minP.y, minP.z}); // 1: Kanan Bawah Depan
        voxelVertices.push_back({maxP.x, minP.y, maxP.z}); // 2: Kanan Bawah Belakang
        voxelVertices.push_back({minP.x, minP.y, maxP.z}); // 3: Kiri Bawah Belakang
        
        // atas
        voxelVertices.push_back({minP.x, maxP.y, minP.z}); // 4: Kiri Atas Depan
        voxelVertices.push_back({maxP.x, maxP.y, minP.z}); // 5: Kanan Atas Depan
        voxelVertices.push_back({maxP.x, maxP.y, maxP.z}); // 6: Kanan Atas Belakang
        voxelVertices.push_back({minP.x, maxP.y, maxP.z}); // 7: Kiri Atas Belakang

        // buat 12 segitiga
        // Sisi Bawah
        voxelFaces.push_back({curIndex + 0, curIndex + 2, curIndex + 1});
        voxelFaces.push_back({curIndex + 0, curIndex + 3, curIndex + 2});
        // Sisi Atas
        voxelFaces.push_back({curIndex + 4, curIndex + 5, curIndex + 6});
        voxelFaces.push_back({curIndex + 4, curIndex + 6, curIndex + 7});
        // Sisi Depan
        voxelFaces.push_back({curIndex + 0, curIndex + 1, curIndex + 5});
        voxelFaces.push_back({curIndex + 0, curIndex + 5, curIndex + 4});
        // Sisi Belakang
        voxelFaces.push_back({curIndex + 3, curIndex + 6, curIndex + 2});
        voxelFaces.push_back({curIndex + 3, curIndex + 7, curIndex + 6});
        // Sisi Kiri
        voxelFaces.push_back({curIndex + 0, curIndex + 4, curIndex + 7});
        voxelFaces.push_back({curIndex + 0, curIndex + 7, curIndex + 3});
        // Sisi Kanan
        voxelFaces.push_back({curIndex + 1, curIndex + 2, curIndex + 6});
        voxelFaces.push_back({curIndex + 1, curIndex + 6, curIndex + 5});
    

    }
    
}
int main(){


    cout << "Masukkan path input (format .obj) : " ;
    string path; cin >> path;
    cout <<endl;
    
    if(loadObject(path, vertices, faces)){

        cout << "SUKSES MEMBACA FILE!" << endl;

        AABB rootBound;
        rootBound.minPoint = {99999.0f, 99999.0f, 99999.0f}; 
        rootBound.maxPoint = {-99999.0f, -99999.0f, -99999.0f}; 
        
        for(auto v : vertices) {
            rootBound.minPoint.x = min(rootBound.minPoint.x, v.x);
            rootBound.minPoint.y = min(rootBound.minPoint.y, v.y);
            rootBound.minPoint.z = min(rootBound.minPoint.z, v.z);
            rootBound.maxPoint.x = max(rootBound.maxPoint.x, v.x);
            rootBound.maxPoint.y = max(rootBound.maxPoint.y, v.y);
            rootBound.maxPoint.z = max(rootBound.maxPoint.z, v.z);
        }

        vector<int> allFaceIds;
        for(int i = 0; i < faces.size(); i++) allFaceIds.push_back(i);
        cout << endl;
        auto start = chrono::high_resolution_clock::now();
        cout << "Masukkan kedalam maksimum dari Octree : " ;
        cin >> maxDepth; 
        cout << endl;

        statTerbentuk.resize(maxDepth+1,0);
        statTidakDitelusuri.resize(maxDepth+1,0);

        Octree* root = new Octree(rootBound, 0);
        root->buildOctree(allFaceIds, faces, vertices, maxDepth);

        buildResult(root);

        // string targetOutput = "test/"+ fileName + "_" + to_string(maxDepth) +"_voxel.obj";
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end-start);

        cout << "Banyak voxel yang terbentuk : " << voxelFaces.size()/12 << endl;
        cout << "Banyak vertex yang terbentuk : " << voxelVertices.size() << endl;
        cout << "Banyak faces yang terbentuk : " << voxelFaces.size() << endl;
        
        cout << endl;
        cout << "Statistik node octree yang terbentuk :" << endl;  
        for(int i = 0 ; i < maxDepth+1; i++) cout << i << " : " << statTerbentuk[i] << endl;

        cout << endl;
        cout << "Statistik node octree yang tidak perlu ditelusuri : " << endl;
        for(int i = 0 ; i < maxDepth+1 ; i++) cout << i << " : " << statTidakDitelusuri[i] << endl;

        cout << endl;
        cout << "Kedalaman Octree : " << finalDepth << endl;
        
        cout << endl;
        cout << "Durasi Proses : " << duration.count() << " ms" << endl;

        cout << endl;
        cout << "Masukkan nama file output : "; 
        string targetOutput; cin >> targetOutput;
        targetOutput = "test/" + targetOutput;
        
        ofstream outFile(targetOutput);
  
        if(outFile.is_open()) {
            for(auto v : voxelVertices){
                outFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
            }
            for(auto f : voxelFaces){
                // .obj dimulai dari 1, jadi kita tambah 1
                outFile << "f " << (f.i + 1) << " " << (f.j + 1) << " " << (f.k + 1) << "\n";
            }
            outFile.close();
            cout << "Berhasil! disimpan di " << targetOutput ;
        } else {
            cout << "Gagal buat output" << endl;
        }
    }
    return 0;
}