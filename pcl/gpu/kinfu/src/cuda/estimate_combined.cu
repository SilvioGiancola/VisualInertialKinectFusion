/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

//#include <pcl/gpu/utils/device/block.hpp>
//#include <pcl/gpu/utils/device/funcattrib.hpp>
#include "device.hpp"

#include <pcl/gpu/containers/device_array.h>

#include <thrust/extrema.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/execution_policy.h>

#include <thrust/device_vector.h>
#include <thrust/transform.h>
#include <thrust/iterator/zip_iterator.h>
#include <stdio.h>

namespace pcl
{
namespace device
{
typedef float float_type;

template<int CTA_SIZE_, typename T>
static __device__ __forceinline__ void reduce(volatile T* buffer)
{
    int tid = Block::flattenedThreadId();
    T val =  buffer[tid];

    if (CTA_SIZE_ >= 1048576) { if (tid < 524288) buffer[tid] = val = val + buffer[tid + 524288]; __syncthreads(); }
    if (CTA_SIZE_ >= 524288) { if (tid < 262144) buffer[tid] = val = val + buffer[tid + 262144]; __syncthreads(); }
    if (CTA_SIZE_ >= 262144) { if (tid < 131072) buffer[tid] = val = val + buffer[tid + 131072]; __syncthreads(); }
    if (CTA_SIZE_ >= 131072) { if (tid < 65536) buffer[tid] = val = val + buffer[tid + 65536]; __syncthreads(); }
    if (CTA_SIZE_ >= 65536) { if (tid < 32768) buffer[tid] = val = val + buffer[tid + 32768]; __syncthreads(); }
    if (CTA_SIZE_ >= 32768) { if (tid < 16384) buffer[tid] = val = val + buffer[tid + 16384]; __syncthreads(); }
    if (CTA_SIZE_ >= 16384) { if (tid < 8192) buffer[tid] = val = val + buffer[tid + 8192]; __syncthreads(); }
    if (CTA_SIZE_ >= 8192) { if (tid < 4096) buffer[tid] = val = val + buffer[tid + 4096]; __syncthreads(); }
    if (CTA_SIZE_ >= 4096) { if (tid < 2048) buffer[tid] = val = val + buffer[tid + 2048]; __syncthreads(); }
    if (CTA_SIZE_ >= 2048) { if (tid < 1024) buffer[tid] = val = val + buffer[tid + 1024]; __syncthreads(); }
    if (CTA_SIZE_ >= 1024) { if (tid < 512) buffer[tid] = val = val + buffer[tid + 512]; __syncthreads(); }
    if (CTA_SIZE_ >=  512) { if (tid < 256) buffer[tid] = val = val + buffer[tid + 256]; __syncthreads(); }
    if (CTA_SIZE_ >=  256) { if (tid < 128) buffer[tid] = val = val + buffer[tid + 128]; __syncthreads(); }
    if (CTA_SIZE_ >=  128) { if (tid <  64) buffer[tid] = val = val + buffer[tid +  64]; __syncthreads(); }

    if (tid < 32)
    {
        if (CTA_SIZE_ >=   64) { buffer[tid] = val = val + buffer[tid +  32]; }
        if (CTA_SIZE_ >=   32) { buffer[tid] = val = val + buffer[tid +  16]; }
        if (CTA_SIZE_ >=   16) { buffer[tid] = val = val + buffer[tid +   8]; }
        if (CTA_SIZE_ >=    8) { buffer[tid] = val = val + buffer[tid +   4]; }
        if (CTA_SIZE_ >=    4) { buffer[tid] = val = val + buffer[tid +   2]; }
        if (CTA_SIZE_ >=    2) { buffer[tid] = val = val + buffer[tid +   1]; }
    }
}

struct Combined
{
    enum
    {
        CTA_SIZE_X = 32,
        CTA_SIZE_Y = 8,
        CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y
    };


    Mat33 Rcurr;
    float3 tcurr;

    PtrStep<float> vmap_curr;
    PtrStep<float> nmap_curr;

    Mat33 Rprev_inv;
    float3 tprev;

    Intr intr;

    PtrStep<float> vmap_g_prev;
    PtrStep<float> nmap_g_prev;

    float distThres;
    float angleThres;

 /*   mutable int *dist_PDF;
    mutable int *dist_CPDF;
    mutable int dist_median_bin;
    mutable float dist_median_value;
    mutable int *Output;

    mutable float RangeDistHist; //m
    int nb_bin;*/

    int cols;
    int rows;
    mutable int* cnt_buffer;
    mutable PtrStep<float_type> dist_array;

    mutable PtrStep<float_type> gbuf;

    __device__ __forceinline__ bool
    search (int x, int y, float3& n, float3& d, float3& s) const
    {
        float3 ncurr;
        ncurr.x = nmap_curr.ptr (y)[x];

        if (isnan (ncurr.x))
            return (false);

        float3 vcurr;
        vcurr.x = vmap_curr.ptr (y       )[x];
        vcurr.y = vmap_curr.ptr (y + rows)[x];
        vcurr.z = vmap_curr.ptr (y + 2 * rows)[x];

        float3 vcurr_g = Rcurr * vcurr + tcurr;

        float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);         // prev camera coo space

        int2 ukr;         //projection
        ukr.x = __float2int_rn (vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx);      //4
        ukr.y = __float2int_rn (vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy);                      //4

        if (ukr.x < 0 || ukr.y < 0 || ukr.x >= cols || ukr.y >= rows || vcurr_cp.z < 0)
            return (false);

        float3 nprev_g;
        nprev_g.x = nmap_g_prev.ptr (ukr.y)[ukr.x];

        if (isnan (nprev_g.x))
            return (false);

        float3 vprev_g;
        vprev_g.x = vmap_g_prev.ptr (ukr.y       )[ukr.x];
        vprev_g.y = vmap_g_prev.ptr (ukr.y + rows)[ukr.x];
        vprev_g.z = vmap_g_prev.ptr (ukr.y + 2 * rows)[ukr.x];

        float dist = norm (vprev_g - vcurr_g);

        dist_array(x,y) = dist;
        if (dist > distThres)
            return (false);

        ncurr.y = nmap_curr.ptr (y + rows)[x];
        ncurr.z = nmap_curr.ptr (y + 2 * rows)[x];

        float3 ncurr_g = Rcurr * ncurr;

        nprev_g.y = nmap_g_prev.ptr (ukr.y + rows)[ukr.x];
        nprev_g.z = nmap_g_prev.ptr (ukr.y + 2 * rows)[ukr.x];

        float sine = norm (cross (ncurr_g, nprev_g));

        if (sine >= angleThres)
            return (false);

/*
        float resolution = RangeDistHist / nb_bin;
        int i_hist = dist / resolution;
        if (i_hist < 0 || i_hist >= nb_bin)
        {}
        else
        {
            atomicAdd(dist_PDF + i_hist, 1);
        }*/
        n = nprev_g;
        d = vprev_g;
        s = vcurr_g;
        return (true);
    }

/*

    // 1 thread only
    __device__ __forceinline__ int
    fill_CPDF_histogram() const
    {

        // int tid = Block::flattenedThreadId();
        // printf ("ThreadId for prefixSum is (should be 0, and appear once) : %d\n", tid);

        dist_CPDF[0] = dist_PDF[0];
        for (int i = 1; i < nb_bin; i++)
        {
            dist_CPDF[i] = dist_CPDF[i-1] + dist_PDF[i];
            //  printf ("   -> %d th element of CFPD : %d\n", i, dist_CPDF[i]);
        }
        return 1;
    }



    // 1000 thread
    __device__ __forceinline__ int
    estimateMedian() const
    {

        int tid = Block::flattenedThreadId();

        if (tid == 0)
        {
            int NbCorrepSoFar = dist_CPDF[tid];
            int NbCorrep_ = dist_CPDF[nb_bin-1];
            if (NbCorrepSoFar >= NbCorrep_/2)
            {
                dist_median_bin = tid;
              //  dist_median_value = tid * RangeDistHist / nb_bin;

                Output[0] = dist_CPDF [ nb_bin-1 ];
                Output[1] = dist_median_bin;
            }
        }
        else if (tid > 0)
        {
            int NbCorrepPrevious = dist_CPDF[tid-1];
            int NbCorrepSoFar = dist_CPDF[tid];
            int NbCorrep_ = dist_CPDF[nb_bin-1];
            if (NbCorrepPrevious < NbCorrep_/2 && NbCorrepSoFar >= NbCorrep_/2)
            {
                dist_median_bin = tid;
               // dist_median_value = tid * RangeDistHist / nb_bin;


                Output[0] = dist_CPDF [ nb_bin-1 ];
                Output[1] = dist_median_bin;

            }
        }


        return 1;

    }

*/
    __device__ __forceinline__ void
    operator () () const
    {
        int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
        int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

        float3 n, d, s;
        bool found_coresp = false;

        if (x < cols && y < rows)
            found_coresp = search (x, y, n, d, s);

        float row[7];

        if (found_coresp)
        {
            *(float3*)&row[0] = cross (s, n);
            *(float3*)&row[3] = n;
            row[6] = dot (n, d - s);
        }
        else
            row[0] = row[1] = row[2] = row[3] = row[4] = row[5] = row[6] = 0.f;


        __shared__ float_type smem[CTA_SIZE];
        int tid = Block::flattenedThreadId ();


        __shared__ int cntmem[CTA_SIZE];

        __syncthreads ();
        if (found_coresp) cntmem[tid] = 1;
        else cntmem[tid] = 0;
        __syncthreads ();

        reduce<CTA_SIZE>(cntmem);
        if (tid == 0)
            cnt_buffer[blockIdx.x + gridDim.x * blockIdx.y] = cntmem[0];


        int shift = 0;
        for (int i = 0; i < 6; ++i)        //rows
        {
#pragma unroll
            for (int j = i; j < 7; ++j)          // cols + b
            {
                __syncthreads ();
                smem[tid] = row[i] * row[j];
                __syncthreads ();

                reduce<CTA_SIZE>(smem);

                if (tid == 0)
                    gbuf.ptr (shift++)[blockIdx.x + gridDim.x * blockIdx.y] = smem[0];
            }
        }
    }
};

__global__ void
combinedKernel (const Combined cs)
{
    cs ();
}
/*
__global__ void
CPDF_Filling (const Combined cs) {
    cs.fill_CPDF_histogram();
}


__global__ void
estimateMedian (const Combined cs) {
    cs.estimateMedian();
}*/

struct TranformReduction
{
    enum
    {
        CTA_SIZE = 512,
        STRIDE = CTA_SIZE,

        B = 6, COLS = 6, ROWS = 6, DIAG = 6,
        UPPER_DIAG_MAT = (COLS * ROWS - DIAG) / 2 + DIAG,
        TOTAL = UPPER_DIAG_MAT + B,

        GRID_X = TOTAL
    };

    PtrStep<float_type> gbuf;
    int length;
    mutable int* cnt_buffer;
    mutable int* output_cnt;
  //  mutable int* cnt;
    mutable float_type* output;
    mutable int out_cnt;

    __device__ __forceinline__ void
    operator () () const
    {
        {
            const float_type *beg = gbuf.ptr (blockIdx.x);
            const float_type *end = beg + length;

            int tid = threadIdx.x;
            //printf("tid = %d\n", tid);

            float_type sum = 0.f;
            for (const float_type *t = beg + tid; t < end; t += STRIDE)
                sum += *t;

            __shared__ float_type smem[CTA_SIZE];

            smem[tid] = sum;
            __syncthreads ();

            reduce<CTA_SIZE>(smem);

            if (tid == 0)
                output[blockIdx.x] = smem[0];
        }
        {
           // int tid = Block::flattenedThreadId();
           // printf("tid : %d\n", tid);
            //printf("blockIdx.x : %d\n", blockIdx.x);
            const int* beg = &(cnt_buffer[blockIdx.x]);
            const int* end = beg + length;

            int tid = threadIdx.x;
           // printf("tid = %d\n", tid);

            int sum = 0.f;
            for (const int * t = beg + tid; t < end; t += STRIDE)
                sum += *t;

            __shared__ int smem[CTA_SIZE];

            smem[tid] = sum;
            __syncthreads ();

            reduce<CTA_SIZE>(smem); //1200 / 300 / 75

          //  cnt[0] = cnt_buffer[0];

            __syncthreads ();
           if (tid == 0)
                output_cnt[blockIdx.x] = smem[0];
        }
    }
};

__global__ void
TransformEstimatorKernel2 (const TranformReduction tr)
{
    tr ();
}
}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::device::estimateCombined (const Mat33& Rcurr, const float3& tcurr, 
                               const MapArr& vmap_curr, const MapArr& nmap_curr,
                               const Mat33& Rprev_inv, const float3& tprev, const Intr& intr,
                               const MapArr& vmap_g_prev, const MapArr& nmap_g_prev,
                               float distThres, float angleThres,
                               DeviceArray2D<float_type>& gbuf, DeviceArray<float_type>& mbuf,
                               float_type* matrixA_host, float_type* vectorB_host)
{
    int cols = vmap_curr.cols ();
    int rows = vmap_curr.rows () / 3;


  /*  //PDF and CPDF of distances
    int nbBin = 1000;
    float RangeDistHist = 0.5f;

    pcl::gpu::DeviceArray<int> dist_PDF;
    pcl::gpu::DeviceArray<int> dist_CPDF;
    int dist_PDF_host[nbBin];
    int dist_CPDF_host[nbBin];
    for (int i = 0; i < nbBin; i++)
    {
        dist_PDF_host[i] = 0;
        dist_CPDF_host[i] = 0;
    }

    dist_PDF.upload(dist_PDF_host, nbBin);
    dist_CPDF.upload(dist_CPDF_host, nbBin);


    pcl::gpu::DeviceArray<int> output_device;
    output_device.create(2);

    // Resutls
    int output[2];
    output[0] = 1;
    output[1] = 1;
    output_device.upload(output,2);
*/


    Combined cs;

    cs.Rcurr = Rcurr;
    cs.tcurr = tcurr;

    cs.vmap_curr = vmap_curr;
    cs.nmap_curr = nmap_curr;

    cs.Rprev_inv = Rprev_inv;
    cs.tprev = tprev;

    cs.intr = intr;

    cs.vmap_g_prev = vmap_g_prev;
    cs.nmap_g_prev = nmap_g_prev;

    cs.distThres = distThres;
    cs.angleThres = angleThres;

    cs.cols = cols;
    cs.rows = rows;

/*
    // PDF and Median Stuff
    cs.nb_bin = nbBin;
    cs.dist_PDF = dist_PDF;
    cs.dist_CPDF = dist_CPDF;
    cs.RangeDistHist = RangeDistHist;
    cs.Output = output_device;
*/
    //////////////////////////////

    dim3 block (Combined::CTA_SIZE_X, Combined::CTA_SIZE_Y);
    dim3 grid (1, 1, 1);
    grid.x = divUp (cols, block.x);
    grid.y = divUp (rows, block.y);

    mbuf.create (TranformReduction::TOTAL);
    if (gbuf.rows () != TranformReduction::TOTAL || gbuf.cols () < (int)(grid.x * grid.y))
        gbuf.create (TranformReduction::TOTAL, grid.x * grid.y);



    pcl::gpu::DeviceArray<int> cnt_buffer;
    cnt_buffer.create (grid.x * grid.y);

    pcl::gpu::DeviceArray2D<float> dist_array;
    dist_array.create (rows, cols);

    cs.dist_array = dist_array;
    cs.cnt_buffer = cnt_buffer;

    cs.gbuf = gbuf;

    combinedKernel<<<grid, block>>>(cs);
    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall(cudaDeviceSynchronize());

 /*   ///////////////
    ///NEW 2017


    // from PCF, get CPDF
    CPDF_Filling<<<1, 1>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());

    // Estiamte Median on CPDF -> 1000 threads
    estimateMedian<<<1, nbBin>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());



    output_device.download(output);


    std::cout << "Median estimation : " << output[1]*RangeDistHist/nbBin << " of bin " << output[1] << std::endl;
    std::cout << "Percentage of matches = " << (output[0]*1.f)/(cols*rows)
              << "(" << output[0] << "/" << cols*rows << ")" << std::endl;
*/


    /////////////////////////////

    TranformReduction tr;
    tr.gbuf = gbuf;
    tr.length = grid.x * grid.y;
    tr.output = mbuf;
    tr.cnt_buffer = cnt_buffer;

    pcl::gpu::DeviceArray<int> output_cnt;
    output_cnt.create (1);
    tr.output_cnt = output_cnt;


    TransformEstimatorKernel2<<<TranformReduction::TOTAL, TranformReduction::CTA_SIZE>>>(tr);
    cudaSafeCall (cudaGetLastError ());
    cudaSafeCall (cudaDeviceSynchronize ());

    float_type host_data[TranformReduction::TOTAL];
    mbuf.download (host_data);



    int host_cnt[1];
    output_cnt.download(host_cnt);
    int number_matches = host_cnt[0] ;

    int shift = 0;
    for (int i = 0; i < 6; ++i)  //rows
        for (int j = i; j < 7; ++j)    // cols + b
        {
            float_type value = host_data[shift++] / (number_matches*1.0f);
            if (j == 6)       // vector b
                vectorB_host[i] = value;
            else
                matrixA_host[j * 6 + i] = matrixA_host[i * 6 + j] = value;

        }

   // printf("number_matches = %f [%d/%d]\n", (number_matches*1.0f) / (cols*rows*1.0f), number_matches, cols*rows );
}
