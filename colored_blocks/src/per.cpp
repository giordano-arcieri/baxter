


int main (int argc, char** argv)
{
    //init ros
    //subscriper to camera
    //set up publishers

}

void call_back()
{
    
    //convert input ros pointcloud to pcl pointcloud
    //filter the z-axes removing very far points and very close points (So that we only see the table and blocks)
    // Perform plane segmentation
    // Extract the largest planar surface
    // convert to rospc publish extracted surface
    // Remove the largest surface from the input cloud
    // convert to rospc publish modified input cloud

}


int main (int argc, char** argv)
{
    // init ros
    // subscribe to pc an publish to mss

    return 0;
}

void call_back()
{
    //cluster and find centroids of input cloud


    //filter out bad potential blocks
    //adjust each point to make it with respect to base

    //only put good potential blocks and missplaced blocks in missplaced blocks
      
    //only put the blocks in if it is a missplaced block
          
}

