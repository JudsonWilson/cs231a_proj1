#!/usr/bin/perl

use strict;
use warnings;
use Getopt::Long;


##### SCRIPT USAGE ##########
# perl filter_cameras.pl -q <length of time window> -num <max number of active tracklets per window> file1 file2 file3...
# where filei is one of the cam_.csv files.
#############################


####################################
# VARIABLES FOR COMMAND LINE FLAGS #
####################################

my $quantum = 10000; # TODO make command line options
my $num_active_tracklets_per_quanta = 20;
my $stats = 0;
my $unfiltered = 0;
my $path_to_files = "../cams with topo/";

GetOptions("q=i" => \$quantum,
           "num=i" => \$num_active_tracklets_per_quanta,
           "stats" => \$stats,
           "unfiltered" => \$unfiltered,
           "path" => \$path_to_files);


################################
# GLOBAL VARIABLES #############
################################

# variables necessary to filter data
my $raw_data = {};
my $filtered_data = {};
my($min_time, $max_time, $max_size) = (-1, -1, -1);
my @quanta = ();


# Variables for keeping statistics
my $ave_length = 0;
my $min_length = 1000000000;
my $max_length = 0;

my $ave_distance_per_time = 0;

my $ave_num_per_quanta;
my $max_num_per_quanta;
my $min_num_per_quanta;
my $size_before_filtering_long;


#############################
# MAIN FUNCTION #############
#############################
&read_raw_data;
# always filter out long results. The length of the longest tracklet is
# extrememly long, and the resulting output file will be too big since 
# all tracklets get padded to this length
&gather_tracklet_stats;
&filter_long_tracklets(1.5*$ave_length);
# if unfiltered data is desired, print the current results
if($unfiltered) {
  &print_data($raw_data);

# otherwise, filter by time slices
} else {
  &build_quanta_list;
  &filter_data_by_active_during_quanta;

# gather statistics again and print them if stats flag present
  if($stats) {
    &gather_tracklet_stats;
    &get_per_quanta_stats;
    &print_stats;
  }
  &print_data($filtered_data);
}
#############################
# END MAIN FUNCTION #########
#############################


############################
# SUBROUTINES ##############
############################

# removes all tracklets that exist for a time longer than
# the passed in threshold argument.
sub filter_long_tracklets {
  my $threshold = shift;
  while( my($key, $val) = each %$raw_data) {
    delete $raw_data->{$key} if $val->[-1][2] - $val->[0][2] > $threshold ;
  }
}

# gets statistics about the number of tracklets active in each quanta
sub get_per_quanta_stats {
  $ave_num_per_quanta = 0;
  $max_num_per_quanta = 0;
  $min_num_per_quanta = 1000000000;
  foreach my $cur (@quanta) {
    if(!defined $cur) {
      $min_num_per_quanta = 0;
      next;
    }
    $ave_num_per_quanta += scalar @$cur;
    $min_num_per_quanta = scalar @$cur if @$cur < $min_num_per_quanta;
    $max_num_per_quanta = scalar @$cur if @$cur > $max_num_per_quanta;
  }
  $ave_num_per_quanta /= scalar @quanta if scalar @quanta;
}

# gathers statistics about the length of tracklets
sub gather_tracklet_stats {
  $ave_length = 0;
  $min_length = 1000000000;
  $max_length = 0;

  foreach (keys %$raw_data) {
    my $tracklet = $raw_data->{$_};
    my $length = $tracklet->[-1][2] - $tracklet->[0][2]; 
    $ave_length += $length;
    $min_length = $length if $length < $min_length;
    $max_length = $length if $length > $max_length;

    my $delta_x = $tracklet->[-1][0] - $tracklet->[0][0];
    my $delta_y = $tracklet->[-1][1] - $tracklet->[0][1];
    my $distance = sqrt($delta_x*$delta_x + $delta_y*$delta_y);
    $ave_distance_per_time += $distance/$length;
    
  }
  $ave_length /= scalar (keys %$raw_data) if scalar (keys %$raw_data);
  $ave_distance_per_time /= scalar (keys %$raw_data) if scalar (keys %$raw_data);
}


# prints all the statistics gathered
sub print_stats {
  print "\nquantum = $quantum\n\n";
  print "total number of tracklets:                           " . $size_before_filtering_long . "\n";
  print "total number of tracklets after removing long ones : " . scalar (keys %$raw_data) . "\n\n";
  print "total length of time: " . ($max_time - $min_time) . "\n\n";
  print "average length of tracklet: $ave_length\n";
  print "max length of tracklet:     $max_length\n";
  print "min length of tracklet:     $min_length\n";
  print "average speed per tracklet: $ave_distance_per_time (inverse = " . (1/$ave_distance_per_time) . "\n\n";
  print "average per quanta:  $ave_num_per_quanta\n";
  print "min per quanta:      $min_num_per_quanta\n";
  print "max per quanta:      $max_num_per_quanta\n\n";
  print "Number of filtered tracklets: " . (scalar keys %$filtered_data) . "\n\n";
}


# reads files and populates $raw_data with the info of every tracklet
sub read_raw_data {
 #foreach my $fname (@ARGV) {
 #  die "bad filename: $fname" unless $fname =~ /(.*\/)*cam(\d+)\.csv/;
 #  my $cam = $2;
    foreach my $cam (@ARGV) {
    my $fname = "${path_to_files}cam$cam.csv";
    open FHANDLE, "<$fname" or die "couldn't open $fname for reading";
    while(<FHANDLE>) {
      my($x, $y, $t, $id) = /(\d*\.?\d*\d+) (\d*\.?\d*\d+) (\d*\.?\d*\d+) (\d*\.?\d*\d+)/;
      push @{$raw_data->{"${cam}_${id}"}}, [$x,$y,$t];
      $min_time = $t if $min_time eq -1 or $t < $min_time;
      $max_time = $t if $t > $max_time;
    }
    close FHANDLE;
    #$raw_data->{$_} = [sort { $a->[2] cmp $b->[2] } @{$raw_data->{$_}}] foreach keys %$raw_data;
  }
  $size_before_filtering_long = scalar (keys %$raw_data);
}

# quantize the data: build quanta list
sub build_quanta_list {
  foreach (keys %$raw_data) {
    my($cam,$id) = /(\d+)_(\d+)/;
    my $tracklet = $raw_data->{$_};
    my $min_quanta  = int ($tracklet->[0][2]/$quantum);
    my $max_quanta  = int ($tracklet->[-1][2]/$quantum);
    foreach my $q ($min_quanta..$max_quanta) {
      push @{$quanta[$q]}, $_;
    }
  }
}

# build filtered data list
sub filter_data_by_active_during_quanta {
  foreach my $cur (@quanta) {
    if(defined $cur and scalar @{$cur} < $num_active_tracklets_per_quanta) {
      $filtered_data->{$_} = $raw_data->{$_} foreach @{$cur};
    }
  }
}

sub print_data {
  my $data = shift;
  #get max tracklet size
  while(my($key, $val) = each %$data) {
    $max_size = scalar @$val if scalar @$val > $max_size;
  }

  my $fname = "data_q_${quantum}_num_${num_active_tracklets_per_quanta}_cams";
  $fname .= "_$_" foreach @ARGV;
  $fname .= '.csv';

  open OUTFILE, ">$fname" or die "Couldn't open $fname for writing";

  #prints out the data, padding with -1's so all tracklets are same length in csv file
  foreach my $key (sort keys %$data) {
    my($cam,$id) = $key =~ /(\d+)_(\d+)/;
    my $size = scalar @{$data->{$key}};
     print OUTFILE "$cam $id $size ";
    foreach my $i (0..$max_size) {
      my @vals = $i > $#{$data->{$key}} ? (-1,-1,-1) : @{$data->{$key}[$i]};
       print OUTFILE "@vals ";
    }
     print OUTFILE "\n";
   }

   close OUTFILE;
}
