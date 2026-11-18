// SPDX-License-Identifier: Apache-2.0
#include "../NvtTouchCore.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct nvt_touch_core core;
static uint32_t rng=12345;
static int noise(int amp){rng=rng*1664525u+1013904223u;return (int)((rng>>16)%(unsigned)(2*amp+1))-amp;}
static void base_frame(int16_t *m,int amp){for(int i=0;i<NVT_NODES;++i)m[i]=(int16_t)(5000+(i%7)*3+noise(amp));}
static void blob(int16_t *m,double row,double col,double amplitude,double sigma){
    for(int r=0;r<NVT_ROWS;++r)for(int c=0;c<NVT_COLUMNS;++c){
        double d2=(r-row)*(r-row)+(c-col)*(c-col);
        m[r*NVT_COLUMNS+c]=(int16_t)(m[r*NVT_COLUMNS+c]+(int)lround(amplitude*exp(-d2/(2*sigma*sigma))));
    }
}
static void settle(void){
    int16_t frame[NVT_NODES];nvt_touch_reset(&core);
    for(unsigned i=0;i<NVT_STARTUP_FRAMES;++i){
        base_frame(frame,2);struct nvt_contact c[4];
        int n=nvt_touch_process(&core,frame,4,c,4);assert(n==-1);
    }
    assert(core.have_reference);
}
static int feed(int16_t *m,struct nvt_contact *out,int max){return nvt_touch_process(&core,m,4,out,max);}
static void lift(void){
    int16_t m[NVT_NODES];struct nvt_contact out[12];
    base_frame(m,2);assert(feed(m,out,12)==0);
}

// Each fingertip converges on its own peak and stays there longer than the dropout grace.
static void pinch_and_pause(const int offsets[][2], int fingers, int start, int stop) {
    int ids[NVT_SLOTS] = {0};
    int16_t matrix[NVT_NODES];
    struct nvt_contact out[NVT_SLOTS];
    settle();
    for (int frame = 0; frame < start - stop + NVT_HOLD_FRAMES + 5; ++frame) {
        int radius = frame < 2 ? start : start - (frame - 1);
        if (radius < stop) radius = stop;
        base_frame(matrix, 0);
        int xs[NVT_SLOTS], ys[NVT_SLOTS];
        for (int k = 0; k < fingers; ++k) {
            int row = 20 + radius * offsets[k][0], col = 30 + radius * offsets[k][1];
            blob(matrix, row, col, 900, .72);
            xs[k] = nvt_touch_column_center(col) * NVT_SUPER_RESOLUTION;
            ys[k] = nvt_touch_row_center(row) * NVT_SUPER_RESOLUTION;
        }
        int count = feed(matrix, out, NVT_SLOTS);
        if (!frame) {assert(count == 0); continue;}
        assert(count == fingers);
        int seen = 0;
        for (int i = 0; i < count; ++i) {
            int nearest = -1;
            long best = __LONG_MAX__;
            for (int k = 0; k < fingers; ++k) {
                long dx = out[i].x - xs[k], dy = out[i].y - ys[k];
                long distance = dx * dx + dy * dy;
                if (distance < best) {best = distance; nearest = k;}
            }
            assert(!(seen & (1 << nearest)));
            seen |= 1 << nearest;
            if (frame == 1) ids[nearest] = out[i].tracking_id;
            assert(ids[nearest] == out[i].tracking_id && out[i].peak > 0);
        }
    }
    lift();
}

int main(void){
    // Startup baseline: -1 for 72 frames, then a quiet frame yields no contacts.
    settle();
    int16_t f[NVT_NODES];struct nvt_contact c[12];
    base_frame(f,2);assert(feed(f,c,12)==0);
    // Reference is the per-node median of the startup frames (within the injected noise).
    for(int i=0;i<NVT_NODES;++i)assert(abs(core.reference[i]-(5000+(i%7)*3))<=2);
    // A frame with the wrong type is ignored.
    assert(nvt_touch_process(&core,f,3,c,12)==0);
    // One finger: reported from the second frame on, near the expected super-resolution centroid, same id.
    int id=0;
    for(int frame=0;frame<4;++frame){
        base_frame(f,2);blob(f,20.0,30.0,900,1.3);int n=feed(f,c,12);
        if(frame==0)assert(n==0);
        else{assert(n==1);assert(abs(c[0].x-nvt_touch_column_center(30)*NVT_SUPER_RESOLUTION)<120);
            assert(abs(c[0].y-nvt_touch_row_center(20)*NVT_SUPER_RESOLUTION)<120);
            if(!id)id=c[0].tracking_id;assert(c[0].tracking_id==id&&c[0].slot>=0);}
    }
    // Sub-pixel position: a blob between columns 30 and 31 lands between their centres once the smoothing has settled.
    int n=0;for(int frame=0;frame<40;++frame){base_frame(f,2);blob(f,20.0,30.5,900,1.3);n=feed(f,c,12);}assert(n==1);
    int lo=nvt_touch_column_center(30)*NVT_SUPER_RESOLUTION,hi=nvt_touch_column_center(31)*NVT_SUPER_RESOLUTION;
    assert(c[0].x>lo+100&&c[0].x<hi-100&&c[0].tracking_id==id);
    // Lift, then a new touch is a new contact (not reported on its first frame).
    lift();
    base_frame(f,2);blob(f,10.0,10.0,900,1.3);assert(feed(f,c,12)==0);
    base_frame(f,2);blob(f,10.0,10.0,900,1.3);assert(feed(f,c,12)==1&&c[0].tracking_id!=id);
    // Two fingers far apart: two contacts, distinct ids and slots.
    settle();
    int ids[2]={0,0};
    for(int frame=0;frame<3;++frame){
        base_frame(f,2);blob(f,8.0,12.0,800,1.3);blob(f,30.0,48.0,700,1.3);n=feed(f,c,12);
        if(frame>=1){assert(n==2&&c[0].tracking_id!=c[1].tracking_id&&c[0].slot!=c[1].slot);
            if(!ids[0]){ids[0]=c[0].tracking_id;ids[1]=c[1].tracking_id;}
            assert((c[0].tracking_id==ids[0]&&c[1].tracking_id==ids[1])||(c[0].tracking_id==ids[1]&&c[1].tracking_id==ids[0]));}
    }
    // Faint blobs (below the noise/peak thresholds) never start a contact.
    settle();
    for(int frame=0;frame<4;++frame){base_frame(f,2);blob(f,20,30,80,1.3);assert(feed(f,c,12)==0);}
    // A blob whose peak sits between the up and down thresholds does not start a contact...
    for(int frame=0;frame<4;++frame){base_frame(f,2);blob(f,20,30,260,1.3);assert(feed(f,c,12)==0);}
    // ...but keeps an established one alive.
    for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,20,30,900,1.3);feed(f,c,12);}
    base_frame(f,2);blob(f,20,30,260,1.3);assert(feed(f,c,12)==1);
    // Palm-sized blobs are not reported.
    settle();
    for(int frame=0;frame<4;++frame){base_frame(f,2);blob(f,20,30,900,6.0);assert(feed(f,c,12)==0);}
    // The output limit is honoured.
    settle();
    for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,5,8,800,1.2);blob(f,5,30,800,1.2);blob(f,5,52,800,1.2);blob(f,30,8,800,1.2);
        n=feed(f,c,2);if(frame>=1)assert(n==2);}
    // Edge mapping formula and monotonicity.
    assert(nvt_touch_map_edge_axis(300,NVT_LONG_AXIS_MAX,0)==600-483);
    assert(nvt_touch_map_edge_axis(300,NVT_SHORT_AXIS_MAX,1)==600-483-1);
    assert(nvt_touch_map_edge_axis(10000,NVT_LONG_AXIS_MAX,0)==10000);
    int previous=-1;for(int v=0;v<NVT_LONG_AXIS_MAX;v+=7){int m=nvt_touch_map_edge_axis(v,NVT_LONG_AXIS_MAX,0);assert(m>=previous-1);previous=m;}
    assert(nvt_touch_map_edge_axis(NVT_LONG_AXIS_MAX-1,NVT_LONG_AXIS_MAX,0)>NVT_LONG_AXIS_MAX-600);
    // A baseline frame of type 2 sets the reference immediately.
    nvt_touch_reset(&core);base_frame(f,0);assert(nvt_touch_process(&core,f,2,c,12)==-1&&core.have_reference);
    // Identifiers are reused once their finger is up: 100 separate strokes never go past the panel's slot count (int-267: an ever
    // growing identifier reached backboardd's path limit after about 33 strokes and every later touch was dropped).
    settle();
    int top=0;
    for(int stroke=0;stroke<100;++stroke){
        for(int frame=0;frame<4;++frame){
            base_frame(f,2);blob(f,10.0+(stroke%5)*5,10.0+(stroke%7)*5,900,1.3);n=feed(f,c,12);
            for(int k=0;k<n;++k)if(c[k].tracking_id>top)top=c[k].tracking_id;
        }
        lift();
    }
    assert(top>=1&&top<=NVT_SLOTS);
    // A resting finger whose centroid wanders by up to 0.15 node (8 px, the noise int-271 measured) reports steps of two pixels at most.
    settle();
    int prev_x=-1,prev_y=-1;
    for(int frame=0;frame<60;++frame){
        base_frame(f,2);blob(f,20.0+0.15*((frame%3)-1),30.0+0.1*(((frame*7)%5)-2),900,1.3);n=feed(f,c,12);
        if(frame<2)continue;
        assert(n==1);
        if(frame>=20)assert(abs(c[0].x-prev_x)<=20&&abs(c[0].y-prev_y)<=20);
        prev_x=c[0].x;prev_y=c[0].y;
    }
    assert(core.jitter_frames>0&&core.smoothed_sum<core.jitter_sum/4);
    // A slow drag (1.25 px a frame, 150 px/s) moves the report every frame and never in jumps (int-273: a rest lock held it, then let
    // it go 20 px at once); it ends within a few pixels of the finger.
    {
        int moved=0;
        for(int frame=0;frame<120;++frame){
            base_frame(f,2);blob(f,20.0,30.0+frame*0.025,900,1.3);n=feed(f,c,12);assert(n==1);
            const int step=c[0].x-prev_x;
            assert(step>=-5&&step<=40);
            if(frame>=40&&step>0)++moved;
            prev_x=c[0].x;
        }
        assert(moved>=70);
        const int truth=(int)((nvt_touch_column_center(32)+0.975*(nvt_touch_column_center(33)-nvt_touch_column_center(32)))*NVT_SUPER_RESOLUTION);
        assert(abs(c[0].x-truth)<80);
    }
    // A fast swipe (half a node a frame, 3000 px/s) is followed within about a frame.
    for(int frame=0;frame<16;++frame){base_frame(f,2);blob(f,20.0,35.0+frame*0.5,900,1.3);n=feed(f,c,12);assert(n==1);}
    {
        const int truth=(nvt_touch_column_center(42)+nvt_touch_column_center(43))*NVT_SUPER_RESOLUTION/2;
        assert(c[0].x<truth&&truth-c[0].x<400);
    }
    {   // it stops there and settles on the finger; lifting off, the blob fades below the down threshold and slides, and the reported
        // position does not follow
        for(int frame=0;frame<60;++frame){base_frame(f,2);blob(f,20.0,42.5,900,1.3);n=feed(f,c,12);assert(n==1);}
        const int still_x=c[0].x;
        assert(abs(still_x-(nvt_touch_column_center(42)+nvt_touch_column_center(43))*NVT_SUPER_RESOLUTION/2)<60);
        for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,20.0+frame*0.4,42.5+frame*0.6,250-frame*10,1.3);n=feed(f,c,12);
            assert(n==1&&c[0].x==still_x);}
    }
    // One finger with two maxima two nodes apart is one contact between them; fingers four nodes apart stay two.
    settle();
    for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,20.0,30.0,900,0.8);blob(f,20.0,32.0,800,0.8);n=feed(f,c,12);}
    {
        const int left=nvt_touch_column_center(30)*NVT_SUPER_RESOLUTION,right=nvt_touch_column_center(32)*NVT_SUPER_RESOLUTION;
        assert(n==1&&core.merges>0&&c[0].x>left&&c[0].x<right);
    }
    settle();
    for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,20.0,30.0,900,0.8);blob(f,20.0,34.0,800,0.8);n=feed(f,c,12);}
    assert(n==2&&core.merges==0);
    // A weak local signal bridges a partial dropout. A clear frame ends the stroke immediately.
    settle();
    for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,20.0,30.0,900,1.3);n=feed(f,c,12);}
    assert(n==1);
    {
        const struct nvt_contact before=c[0];
        for(int k=0;k<NVT_HOLD_FRAMES;++k){
            base_frame(f,2);blob(f,20.0,30.0,180,1.3);n=feed(f,c,12);
            assert(n==1&&c[0].tracking_id==before.tracking_id&&c[0].x==before.x&&c[0].y==before.y);
        }
        base_frame(f,2);blob(f,20.0,30.0,900,1.3);n=feed(f,c,12);
        assert(n==1&&c[0].tracking_id==before.tracking_id&&core.holds_bridged==1);
        lift();
        assert(core.ends[NVT_MISS_NONE]==1);
    }
    // Rapid taps on different rows. A faint peak elsewhere in the quiet scan
    // must not join the new key to the old path.
    base_frame(f,0);blob(f,15,24,900,.72);assert(feed(f,c,12)==0);
    assert(feed(f,c,12)==1);
    base_frame(f,0);blob(f,30,48,150,.72);assert(feed(f,c,12)==0);
    base_frame(f,0);blob(f,13,24,900,.72);assert(feed(f,c,12)==0);
    assert(feed(f,c,12)==1 && abs(c[0].y-nvt_touch_row_center(13)*10)<20);
    lift();
    // A second finger keeps scanning active between two nearby keystrokes.
    // The missing key may be held briefly but must never move to the new row.
    base_frame(f,0);blob(f,15,24,900,.72);blob(f,30,48,900,.72);
    assert(feed(f,c,12)==0 && feed(f,c,12)==2);
    int key_id = c[c[0].y < c[1].y ? 0 : 1].tracking_id;
    base_frame(f,0);blob(f,30,48,900,.72);assert(feed(f,c,12)==2);
    blob(f,13,24,900,.72);
    for (int scan=0; scan<=NVT_HOLD_FRAMES; ++scan) {
        n=feed(f,c,12);
        for (int i=0; i<n; ++i)
            if (c[i].tracking_id==key_id) assert(abs(c[i].y-nvt_touch_row_center(15)*10)<20);
    }
    assert(n==2);
    int new_key = c[0].y < c[1].y ? 0 : 1;
    assert(c[new_key].tracking_id!=key_id && abs(c[new_key].y-nvt_touch_row_center(13)*10)<20);
    lift();
    // During the two-scan debounce a swipe has already left the 48 px edge strip.
    base_frame(f,0);blob(f,0,30,900,.8);assert(feed(f,c,12)==0);
    base_frame(f,0);blob(f,2.5,30,900,.8);assert(feed(f,c,12)==1 && c[0].y<480);
    assert(feed(f,c,12)==1 && c[0].y>480);
    lift();
    static const int corners[][2] = {{-1,-1},{-1,1},{1,-1},{1,1}};
    static const int cross[][2] = {{0,0},{-1,0},{1,0},{0,-1},{0,1}};
    pinch_and_pause(corners, 4, 4, 1);
    pinch_and_pause(cross, 5, 6, 2);
    // A finger whose blob turns palm-sized is held, then lifted with the palm reason.
    for(int frame=0;frame<3;++frame){base_frame(f,2);blob(f,20.0,30.0,900,1.3);feed(f,c,12);}
    for(int k=0;k<=NVT_HOLD_FRAMES;++k){base_frame(f,2);blob(f,20.0,30.0,900,6.0);n=feed(f,c,12);}
    assert(n==0&&core.ends[NVT_MISS_PALM]==1);
    // Thirty faint maxima above the noise floor (more than the peak table holds) in the rows before two fingers: both fingers are found.
    settle();
    for(int frame=0;frame<3;++frame){
        base_frame(f,2);
        for(int k=0;k<30;++k)blob(f,2.0+(k/10)*4,3.0+(k%10)*5,160,0.5);
        blob(f,30.0,15.0,900,1.3);blob(f,34.0,45.0,900,1.3);n=feed(f,c,12);
    }
    assert(n==2&&core.peak_overflows>0&&core.peaks_max>=24);
    // A panel whose raw values drift up by 400 over a few thousand frames keeps no maxima: the reference follows (int-274), and a
    // finger is still found afterwards.
    settle();
    for(int frame=0;frame<4000;++frame){
        base_frame(f,2);for(int i=0;i<NVT_NODES;++i)f[i]=(int16_t)(f[i]+frame/10+((i*13)%17==0 ? frame/20 : 0));
        n=feed(f,c,12);assert(n==0);
    }
    {
        const unsigned long before=core.peaks_max;
        base_frame(f,2);for(int i=0;i<NVT_NODES;++i)f[i]=(int16_t)(f[i]+400+((i*13)%17==0 ? 200 : 0));n=feed(f,c,12);
        assert(n==0&&core.reference_frames>3000);(void)before;
        core.peaks_max=0;
        for(int frame=0;frame<3;++frame){base_frame(f,2);for(int i=0;i<NVT_NODES;++i)f[i]=(int16_t)(f[i]+400+((i*13)%17==0 ? 200 : 0));
            blob(f,20.0,30.0,900,1.3);n=feed(f,c,12);}
        assert(n==1&&core.peaks_max<=2);
    }
    // A swipe up from the bottom edge that speeds up to five nodes a frame (about 300 mm/s more than the tracking radius allows from
    // the last position) keeps its finger: tracking looks where the finger is heading (int-275: 13 of 125 strokes ended "far").
    settle();
    {
        double row=38.0;int id=-1;
        for(int frame=0;frame<9;++frame){
            base_frame(f,2);blob(f,row,30.0,900,1.3);n=feed(f,c,12);
            if(frame>=1){assert(n==1);if(id<0)id=c[0].tracking_id;assert(c[0].tracking_id==id);}
            row-=frame<5 ? frame+1.0 : 5.0;
            if(row<2.0)break;
        }
        assert(core.ends[NVT_MISS_FAR]==0);
    }
    lift();
    // Two fingers four nodes apart sliding sideways by two nodes a frame keep their identifiers: the closest pair is matched first, so
    // the leading finger's slot does not take the trailing finger's blob.
    settle();
    {
        int left_id=-1,right_id=-1;
        for(int frame=0;frame<12;++frame){
            base_frame(f,2);blob(f,20.0,10.0+frame*2.0,900,1.3);blob(f,20.0,14.0+frame*2.0,900,1.3);n=feed(f,c,12);
            if(frame<1)continue;
            assert(n==2);
            const int l=c[0].x<c[1].x ? 0 : 1;
            if(left_id<0){left_id=c[l].tracking_id;right_id=c[1-l].tracking_id;}
            assert(c[l].tracking_id==left_id&&c[1-l].tracking_id==right_id);
        }
        assert(core.ends[NVT_MISS_TAKEN]==0&&core.ends[NVT_MISS_FAR]==0);
    }
    // Panel drift while a resting finger is held down: the reference waits (a hand changes far nodes too, see below), the finger
    // keeps its place and identifier, and the reference catches up with the drift in the quiet second after the lift.
    settle();
    {
        const int center_x=nvt_touch_column_center(30)*NVT_SUPER_RESOLUTION;
        const int center_y=nvt_touch_row_center(20)*NVT_SUPER_RESOLUTION;
        const int16_t edge_before=core.reference[4];
        int held_id=-1;
        for(int frame=0;frame<600;++frame){
            const int drift=frame*80/600;
            base_frame(f,2);
            for(int i=0;i<NVT_NODES;++i)f[i]=(int16_t)(f[i]+drift);
            blob(f,20.0,30.0,900,1.3);
            n=feed(f,c,12);
            if(frame==0){assert(n==0);continue;}
            assert(n==1);
            if(held_id<0)held_id=c[0].tracking_id;
            assert(c[0].tracking_id==held_id);
            assert(abs(c[0].x-center_x)<80&&abs(c[0].y-center_y)<80);
        }
        assert(core.reference[4]==edge_before);
        for(int frame=0;frame<120;++frame){
            base_frame(f,2);
            for(int i=0;i<NVT_NODES;++i)f[i]=(int16_t)(f[i]+80);
            assert(feed(f,c,12)==0);
        }
        assert(abs(core.reference[4]-(5000+(4%7)*3+80))<=3);
        assert(abs(core.reference[20*NVT_COLUMNS+30]-(5000+((20*NVT_COLUMNS+30)%7)*3+80))<=3);
    }
    // Post-doze thermal hotspots (an uncoupled node at row 0 col 4 above NVT_DOWN_THRESHOLD and a warm patch above NVT_PEAK_THRESHOLD)
    // neither spawn a ghost Dock tap during normal taps elsewhere on the panel nor keep the reference from converging after them.
    settle();
    {
        const int tap_x=nvt_touch_column_center(45)*NVT_SUPER_RESOLUTION;
        const int tap_y=nvt_touch_row_center(13)*NVT_SUPER_RESOLUTION;
        for(int tap=0;tap<4;++tap){
            for(int frame=0;frame<36;++frame){
                base_frame(f,2);
                f[4]=(int16_t)(f[4]+340);
                f[3]=(int16_t)(f[3]+85);
                blob(f,32.0,22.0,250,1.1);
                if(frame<6)blob(f,13.0,45.0,900,1.2);
                n=feed(f,c,12);
                if(frame==0)assert(n==0);
                else if(frame<6){
                    assert(n==1);
                    assert(abs(c[0].x-tap_x)<120&&abs(c[0].y-tap_y)<120);
                }else{
                    assert(n==0);
                }
            }
        }
        // The three quiet seconds before the controller is sent back to doze: the reference has taken both in by then.
        for(int frame=0;frame<360;++frame){
            base_frame(f,2);
            f[4]=(int16_t)(f[4]+340);
            f[3]=(int16_t)(f[3]+85);
            blob(f,32.0,22.0,250,1.1);
            assert(feed(f,c,12)==0);
        }
        assert(abs(core.reference[4]-(5000+(4%7)*3+340))<=15);
        assert(abs(core.reference[32*NVT_COLUMNS+22]-(5000+((32*NVT_COLUMNS+22)%7)*3+250))<=15);
    }
    // A wide, weak rise is no hand: as large as a palm (about 90 nodes above the noise floor) but below the peak threshold, the way a
    // panel that warmed up while the controller dozed shows it. Nothing is reported and the reference takes it in.
    settle();
    {
        for(int frame=0;frame<360;++frame){
            base_frame(f,2);
            blob(f,20.0,30.0,150,6.0);
            assert(feed(f,c,12)==0);
        }
        assert(abs(core.reference[20*NVT_COLUMNS+30]-(5000+((20*NVT_COLUMNS+30)%7)*3+150))<=5);
    }
    // The reference waits NVT_REFERENCE_SETTLE frames after the last finger before it moves again: what a lifting finger still shows
    // (here 200, below the up threshold) is not written into it.
    settle();
    {
        const int node=20*NVT_COLUMNS+30;
        for(int frame=0;frame<10;++frame){base_frame(f,2);blob(f,20.0,30.0,900,1.3);feed(f,c,12);}
        const int16_t before=core.reference[node];
        for(int frame=0;frame<NVT_REFERENCE_SETTLE;++frame){base_frame(f,2);blob(f,20.0,30.0,200,1.3);feed(f,c,12);}
        assert(core.reference[node]==before);
    }
    // What the panel shows while fingers are down is not written into the reference. Patches far from two held fingers read 350 below
    // their rest value for as long as the fingers stay and return to it when they lift: nothing is reported after the lift, and no
    // contact is left behind. (On hardware, a reference that kept moving during the touch left ten contacts that never lifted.)
    settle();
    {
        static const int far[8][2]={{8,50},{31,11},{5,20},{34,40},{12,8},{36,25},{3,35},{22,54}};
        for(int frame=0;frame<400;++frame){
            base_frame(f,2);
            blob(f,20.0,30.0,900,1.3);blob(f,26.0,44.0,900,1.3);
            for(int p=0;p<8;++p)blob(f,far[p][0],far[p][1],-350,1.2);
            n=feed(f,c,12);
            if(frame>0)assert(n==2);
        }
        for(int frame=0;frame<120;++frame){
            base_frame(f,2);
            assert(feed(f,c,12)==0);
        }
        for(int s=0;s<NVT_SLOTS;++s)assert(!core.slots[s].active);
    }
    puts("PASS: NvtTouchCore baseline, tracking, palm, smoothing, partial dropout, fast retap, edge onset, four/five-finger pinch and pause, overflow, drift, warm nodes and far-field of held fingers; synthetic frames");
    return 0;
}
