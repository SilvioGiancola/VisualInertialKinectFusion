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

#include "device.hpp"
//#include <pcl/gpu/utils/device/block.hpp>
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
__device__ unsigned int count = 0;

struct CorespSearch
{
    enum { CTA_SIZE_X = 32, CTA_SIZE_Y = 8, CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y };

    struct plus
    {
        __forceinline__ __device__ int
        operator () (const int &lhs, const volatile int& rhs) const {
            return lhs + rhs;
        }
    };


    PtrStep<float> vmap_g_curr;
    PtrStep<float> nmap_g_curr;

    Mat33 Rprev_inv;
    float3 tprev;

    Intr intr;

    PtrStep<float> vmap_g_prev;
    PtrStep<float> nmap_g_prev;

    float distThres;
    float angleThres;

    mutable int *dist_PDF;
    mutable int *dist_CPDF;
    mutable int dist_median_bin;
    mutable float dist_median_value;
    mutable int *Output;

    mutable float RangeDistHist; //m
    float range_around_median; //m

    mutable PtrStepSz<short2> coresp;
    mutable PtrStepSz<float> dist_matr;

    mutable int* gbuf;
    int nb_bin;




    mutable PtrStepSz<bool> error_spotted;



    __device__ __forceinline__ int
    estimateDistances() const
    {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= coresp.cols || y >= coresp.rows)
            return 0;

        error_spotted.ptr(y)[x] = true;
        dist_matr.ptr (y)[x] = -1;
        coresp.ptr (y)[x] = make_short2 (-1, -1);

        float3 ncurr_g;
        ncurr_g.x = nmap_g_curr.ptr (y)[x];

        if (isnan (ncurr_g.x))
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }
        float3 vcurr_g;
        vcurr_g.x = vmap_g_curr.ptr (y              )[x];
        vcurr_g.y = vmap_g_curr.ptr (y + coresp.rows)[x];
        vcurr_g.z = vmap_g_curr.ptr (y + 2 * coresp.rows)[x];

        float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);         // prev camera coo space

        int2 ukr;         //projection
        ukr.x = __float2int_rn (vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx);      //4
        ukr.y = __float2int_rn (vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy);                      //4

        if (ukr.x < 0 || ukr.y < 0 || ukr.x >= coresp.cols || ukr.y >= coresp.rows)
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }

        float3 nprev_g;
        nprev_g.x = nmap_g_prev.ptr (ukr.y)[ukr.x];

        if (isnan (nprev_g.x))
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }

        float3 vprev_g;
        vprev_g.x = vmap_g_prev.ptr (ukr.y              )[ukr.x];
        vprev_g.y = vmap_g_prev.ptr (ukr.y + coresp.rows)[ukr.x];
        vprev_g.z = vmap_g_prev.ptr (ukr.y + 2 * coresp.rows)[ukr.x];


        float dist = norm (vcurr_g - vprev_g);
        dist_matr.ptr (y)[x] = dist;




        ncurr_g.y = nmap_g_curr.ptr (y + coresp.rows)[x];
        ncurr_g.z = nmap_g_curr.ptr (y + 2 * coresp.rows)[x];


        nprev_g.y = nmap_g_prev.ptr (ukr.y + coresp.rows)[ukr.x];
        nprev_g.z = nmap_g_prev.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

        float sine = norm (cross (ncurr_g, nprev_g));

        /*if (sine >= 1 || asinf(sine) >= angleThres)
            return 0;*/

        if (/*sine >= 1 || */ sine >= angleThres)
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }


        error_spotted.ptr(y)[x] = false;
        return 1;
    }


    __device__ __forceinline__ int
    fill_PDF_histogram() const
    {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;
        if (x >= coresp.cols || y >= coresp.rows)
            return 0;

        if ( error_spotted.ptr(y)[x])
            return 0;

        float resolution = RangeDistHist / nb_bin;

        int i_hist = dist_matr.ptr (y)[x] / resolution;

        if (i_hist < 0 || i_hist >= nb_bin)
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }
        atomicAdd(dist_PDF + i_hist, 1);

        return 1;
    }



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

                int bin_end = dist_median_bin + __float2int_rn(range_around_median * nb_bin / RangeDistHist);

                Output[0] = dist_CPDF [ bin_end ];
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
                dist_median_value = tid * RangeDistHist / nb_bin;
                //printf ("Median estimation : %f of bin %d\n", dist_median_value, dist_median_bin);
                int bin_init = dist_median_bin - __float2int_rn(range_around_median * nb_bin / RangeDistHist);
                int bin_end = dist_median_bin + __float2int_rn(range_around_median * nb_bin / RangeDistHist);

                if (bin_init < 0) bin_init = 0;
                if (bin_end > nb_bin-1) bin_end = nb_bin-1;
                Output[0] = dist_CPDF [ bin_end ] - dist_CPDF [ bin_init ];
                Output[1] = dist_median_bin;

            }
        }
        return 1;

    }


    // 300k thread
    __device__ __forceinline__ int
    findFilteredCorrespondences () const
    {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;


        /// HERE I CAN ADD A FILTERING BASED ON % OF POINT I HAVE LEFT!!
        ///
        /*  if (Output[0] < 0.1 * dist_CPDF[nb_bin-1])
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }*/



        if ( error_spotted.ptr(y)[x])
            return 0;


        if (dist_matr.ptr (y)[x] < 0)
        {

            printf("    ->    How come dist is negatif and the error has not been spotted yet??   -->> PLEASE DEBUG ME IF YOU SEE THIS!!!!");
            error_spotted.ptr(y)[x] = true;
            return 0;
        }



        if (dist_matr.ptr (y)[x] > dist_median_value + range_around_median)
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }


        if (dist_matr.ptr (y)[x] < dist_median_value - range_around_median)
        {
            error_spotted.ptr(y)[x] = true;
            return 0;
        }





        float3 vcurr_g;
        vcurr_g.x = vmap_g_curr.ptr (y              )[x];
        vcurr_g.y = vmap_g_curr.ptr (y + coresp.rows)[x];
        vcurr_g.z = vmap_g_curr.ptr (y + 2 * coresp.rows)[x];

        float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);         // prev camera coo space
        int2 ukr;         //projection
        ukr.x = __float2int_rn (vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx);      //4
        ukr.y = __float2int_rn (vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy);                      //4



        coresp.ptr (y)[x] = make_short2 (ukr.x, ukr.y);


        return 1;
    }


    __device__ __forceinline__ int
    search () const
    {

        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= coresp.cols || y >= coresp.rows)
            return 0;

        coresp.ptr (y)[x] = make_short2 (-1, -1);

        float3 ncurr_g;
        ncurr_g.x = nmap_g_curr.ptr (y)[x];

        if (isnan (ncurr_g.x))
            return 0;

        float3 vcurr_g;
        vcurr_g.x = vmap_g_curr.ptr (y              )[x];
        vcurr_g.y = vmap_g_curr.ptr (y + coresp.rows)[x];
        vcurr_g.z = vmap_g_curr.ptr (y + 2 * coresp.rows)[x];

        float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);         // prev camera coo space

        int2 ukr;         //projection
        ukr.x = __float2int_rn (vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx);      //4
        ukr.y = __float2int_rn (vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy);                      //4

        if (ukr.x < 0 || ukr.y < 0 || ukr.x >= coresp.cols || ukr.y >= coresp.rows)
            return 0;

        float3 nprev_g;
        nprev_g.x = nmap_g_prev.ptr (ukr.y)[ukr.x];

        if (isnan (nprev_g.x))
            return 0;

        float3 vprev_g;
        vprev_g.x = vmap_g_prev.ptr (ukr.y              )[ukr.x];
        vprev_g.y = vmap_g_prev.ptr (ukr.y + coresp.rows)[ukr.x];
        vprev_g.z = vmap_g_prev.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

        float dist = norm (vcurr_g - vprev_g);
        if (dist > distThres)
            return 0;

        ncurr_g.y = nmap_g_curr.ptr (y + coresp.rows)[x];
        ncurr_g.z = nmap_g_curr.ptr (y + 2 * coresp.rows)[x];

        nprev_g.y = nmap_g_prev.ptr (ukr.y + coresp.rows)[ukr.x];
        nprev_g.z = nmap_g_prev.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

        float sine = norm (cross (ncurr_g, nprev_g));

        /*if (sine >= 1 || asinf(sine) >= angleThres)
                    return 0;*/

        if (/*sine >= 1 || */ sine >= angleThres)
            return 0;

        coresp.ptr (y)[x] = make_short2 (ukr.x, ukr.y);
        return 1;
    }
/*
    __device__ __forceinline__ void
    operator () () const
    {
        int mask = search ();

    }*/
};

__global__ void
corespKernel (const CorespSearch cs) {
    cs.search();
}

__global__ void
corespFinding (const CorespSearch cs) {
    cs.estimateDistances();
}

__global__ void
corespFiltering (const CorespSearch cs) {
    cs.findFilteredCorrespondences();
}

__global__ void
histFilling (const CorespSearch cs) {
    cs.fill_PDF_histogram();
}

__global__ void
CPDF_Filling (const CorespSearch cs) {
    cs.fill_CPDF_histogram();
}


__global__ void
estimateMedian (const CorespSearch cs) {
    cs.estimateMedian();
}



}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::device::findCoresp (const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, 
                         const Mat33& Rprev_inv, const float3& tprev, const Intr& intr,
                         const MapArr& vmap_g_prev, const MapArr& nmap_g_prev,
                         float distThres, float angleThres, PtrStepSz<short2> coresp/*, float* median*/)
{
    // Matrix of distances
    pcl::gpu::DeviceArray2D<float> dist_matr;
    dist_matr.create(coresp.rows, coresp.cols);



    //PDF and CPDF of distances
    int nbBin = 1000;
    float RangeDistHist = 0.5f;
    float range_around_median = 0.1f;



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

    // Matrix of error bool
    pcl::gpu::DeviceArray2D<bool> error_spotted;
    error_spotted.create(coresp.rows, coresp.cols);



    CorespSearch cs;

    cs.vmap_g_curr = vmap_g_curr;
    cs.nmap_g_curr = nmap_g_curr;

    cs.Rprev_inv = Rprev_inv;
    cs.tprev = tprev;

    cs.intr = intr;

    cs.vmap_g_prev = vmap_g_prev;
    cs.nmap_g_prev = nmap_g_prev;

    cs.distThres = distThres;
    cs.angleThres = angleThres;

    cs.coresp = coresp;

    cs.nb_bin = nbBin;
    cs.dist_PDF = dist_PDF;
    cs.dist_CPDF = dist_CPDF;
    cs.dist_matr = dist_matr;
    cs.error_spotted = error_spotted;

    cs.range_around_median = range_around_median;
    cs.RangeDistHist = RangeDistHist;



    pcl::gpu::DeviceArray<int> output_device;
    output_device.create(2);
    // output_device[0] = coresp.rows * coresp.cols;
    cs.Output = output_device;

    dim3 block (CorespSearch::CTA_SIZE_X, CorespSearch::CTA_SIZE_Y);
    dim3 grid (divUp (coresp.cols, block.x), divUp (coresp.rows, block.y));



    // Resutls
    int output[2];
    output[0] = 1;
    output[1] = 1;



    /*
    corespKernel<<<grid, block>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall (cudaDeviceSynchronize ());
*/





    corespFinding<<<grid, block>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());


    //300k thread -> each pixel/dist
    histFilling<<<grid, block>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());


    // from PCF, get CPDF
    CPDF_Filling<<<1, 1>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());

    // Estiamte Median on CPDF -> 1000 threads
    estimateMedian<<<1, nbBin>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());


    // Filter correspondences -> 300k thread
    corespFiltering<<<grid, block>>>(cs);

    cudaSafeCall ( cudaGetLastError () );
    cudaSafeCall ( cudaDeviceSynchronize ());


    output_device.download(output);

  //  median[0] = output[1]*RangeDistHist/nbBin;
    std::cout << "Median estimation : " << output[1]*RangeDistHist/nbBin << " of bin " << output[1] << std::endl;
    std::cout << "Percentage of matches = " << (output[0]*1.f)/(coresp.cols*coresp.rows)
              << "(" << output[0] << "/" << coresp.cols*coresp.rows << ")" << std::endl;


    return output[1];

}
