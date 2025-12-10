import type {ReactNode, Fragment} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string; // Using Unicode icons or custom icons
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Complete Learning Path',
    icon: '📚',
    description: (
      <>
        From ROS 2 fundamentals to advanced Vision-Language-Action systems,
        this comprehensive curriculum covers all aspects of humanoid robotics.
      </>
    ),
  },
  {
    title: 'Hands-On Implementation',
    icon: '⚙️',
    description: (
      <>
        Practical code examples and runnable implementations with verification
        procedures for each concept and module.
      </>
    ),
  },
  {
    title: 'Industry-Ready Skills',
    icon: '🤖',
    description: (
      <>
        Learn cutting-edge technologies including NVIDIA Isaac, Unity, Gazebo,
        and real-world deployment strategies.
      </>
    ),
  },
  {
    title: 'Integrated Systems',
    icon: '🔗',
    description: (
      <>
        Master the complete pipeline: VOICE ⟶ PLAN ⟶ NAVIGATE ⟶ RECOGNIZE OBJECT ⟶ MANIPULATE
      </>
    ),
  },
  {
    title: 'Capstone Project',
    icon: '🎯',
    description: (
      <>
        Build an autonomous humanoid system that demonstrates all learned concepts
        in a real-world application.
      </>
    ),
  },
  {
    title: 'Academic Rigor',
    icon: '🎓',
    description: (
      <>
        Comprehensive reference materials, APA citations, and research-backed
        content for serious learners.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureIcon}>
          {icon}
        </div>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container padding-vert--lg">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
