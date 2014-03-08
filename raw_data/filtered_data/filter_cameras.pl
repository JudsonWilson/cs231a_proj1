#!/usr/bin/perl

use strict;
use warnings;
use Getopt::Long;

my $quantum = 10000; # TODO make command line options
my $num_active_tracklets_per_quanta = 20;
my $stats = 0;

GetOptions("q=i" => \$quantum,
           "num=i" => \$num_active_tracklets_per_quanta,
           "stats" => \$stats);


# usage: my @tracklet = @{$raw_data->{"${cam}_${trk_id"}}
# usage: my($x,$y,$t) = $raw_data->{"${cam}_${trk_id}"}[i]
my $raw_data = {};
my $filtered_data = {};

my($min_time, $max_time, $max_size) = (-1, -1, -1);

my @quanta = ();

my $ave_length = 0;
my $min_length = 1000000000;
my $max_length = 0;

my $ave_num_per_quanta;
my $max_num_per_quanta;
my $min_num_per_quanta;
my $size_before_filtering_long;

# main function

&read_raw_data;
&gather_tracklet_stats;
&filter_long_tracklets(1.5*$ave_length);
&gather_tracklet_stats;
&build_quanta_list;
&filter_data_by_active_during_quanta;
if($stats) {
  &get_per_quanta_stats;
  &print_stats;
} else {
  &print_filtered_data;
}

# end main function

sub filter_long_tracklets {
  my $threshold = shift;
  $size_before_filtering_long = scalar (keys %$raw_data);
  while( my($key, $val) = each %$raw_data) {
    delete $raw_data->{$key} if $val->[-1][2] - $val->[0][2] > $threshold ;
  }
}

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
  $ave_num_per_quanta /= scalar @quanta;
}

sub print_stats {
  print "\nquantum = $quantum\n\n";
  print "total number of tracklets:                           " . $size_before_filtering_long . "\n";
  print "total number of tracklets after removing long ones : " . scalar (keys %$raw_data) . "\n\n";
  print "total length of time: " . ($max_time - $min_time) . "\n\n";
  print "average length of tracklet: $ave_length\n";
  print "max length of tracklet:     $max_length\n";
  print "min length of tracklet:     $min_length\n\n";
  print "average per quanta:  $ave_num_per_quanta\n";
  print "min per quanta:      $min_num_per_quanta\n";
  print "max per quanta:      $max_num_per_quanta\n\n";
}


# reads files and populates $raw_data
sub read_raw_data {
  foreach my $fname (@ARGV) {
    die "bad filename: $fname" unless $fname =~ /(.*\/)*cam(\d+)\.csv/;
    my $cam = $2;
    open FHANDLE, "<$fname" or die "couldn't open $fname for reading";
    while(<FHANDLE>) {
      my($x, $y, $t, $id) = /(\d+) (\d+) (\d+) (\d+)/;
      push @{$raw_data->{"${cam}_${id}"}}, [$x,$y,$t];
      $min_time = $t if $min_time eq -1 or $t < $min_time;
      $max_time = $t if $t > $max_time;
    }
    close FHANDLE;
  }
}

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
  }
  $ave_length /= scalar (keys %$raw_data);
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

sub print_filtered_data {
# get max tracklet size
  while(my($key, $val) = each %$filtered_data) {
    $max_size = scalar @$val if scalar @$val > $max_size;
  }

# prints out the data, padding with -1's
  foreach my $key (keys %$filtered_data) {
    my($cam,$id) = $key =~ /(\d+)_(\d+)/;
    my $size = scalar @{$filtered_data->{$key}};
    print "$cam $id $size ";
    foreach my $i (0..$max_size) {
      my @vals = $i > $#{$filtered_data->{$key}} ? (-1,-1,-1) : @{$filtered_data->{$key}[$i]};
      print "@vals ";
    }
    print "\n";
  }
}
