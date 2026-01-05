#!/usr/bin/env python3
"""
Parameter merge utility for loading and combining YAML configuration files.
Supports hierarchical configuration with override capability.
"""

import yaml
import argparse
import os
from pathlib import Path


def merge_configs(base, override):
    """
    Recursively merge two YAML configurations.
    Values in override will override values in base.
    """
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override if override is not None else base
    
    merged = base.copy()
    
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = merge_configs(merged[key], value)
        else:
            merged[key] = value
    
    return merged


def load_yaml(file_path):
    """Load YAML file."""
    if not os.path.exists(file_path):
        print(f"Warning: Config file not found: {file_path}")
        return {}
    
    with open(file_path, 'r') as f:
        return yaml.safe_load(f) or {}


def save_yaml(data, file_path):
    """Save YAML file."""
    with open(file_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(
        description='Merge YAML configuration files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Merge common + vehicle + controller configs
  %(prog)s -b config/common/actuator.yaml -o config/loader_v1/actuator.yaml -o config/controllers/pid.yaml -o merged.yaml
        """
    )
    
    parser.add_argument(
        '-b', '--base',
        required=True,
        help='Base YAML configuration file'
    )
    
    parser.add_argument(
        '--override',
        action='append',
        default=[],
        dest='overrides',
        help='Override YAML configuration file (can be used multiple times)'
    )
    
    parser.add_argument(
        '-o', '--output',
        required=True,
        help='Output merged YAML file'
    )
    
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    # Load base config
    base_config = load_yaml(args.base)
    
    if args.verbose:
        print(f"Loaded base config from: {args.base}")
    
    # Load and merge override configs
    merged_config = base_config
    for override_file in args.overrides:
        override_config = load_yaml(override_file)
        merged_config = merge_configs(merged_config, override_config)
        
        if args.verbose:
            print(f"Applied override from: {override_file}")
    
    # Save merged config
    save_yaml(merged_config, args.output)
    
    if args.verbose:
        print(f"Saved merged config to: {args.output}")
    else:
        print(f"Merged configuration saved to: {args.output}")


if __name__ == '__main__':
    main()

